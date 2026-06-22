#!/usr/bin/env python3
"""
map_editor_node.py — chạy trên ROBOT tại /home/pan/ros2_ws/src/mobile/

Topics:
  Sub  /map_editor/select           std_msgs/String      — tên map cần load
  Pub  /map_editor/image            std_msgs/String      — JSON {image b64PNG, width, height, resolution, origin_x, origin_y, annotations}
  Pub  /map_editor/maps_list        std_msgs/String      — JSON array tên map
  Sub  /map_editor/save                  std_msgs/String      — JSON {name, image:b64PNG, annotations:[...]}
  Pub  /map_editor/save_result           std_msgs/String      — "ok:{name}" hoặc "err:{msg}"
  Sub  /map_editor/set_nav_launch        std_msgs/String      — tên map cần ghi vào nav2.launch.py
  Pub  /map_editor/set_nav_launch_result std_msgs/String      — "ok:{name}" hoặc "err:{msg}"
  Sub  /map_editor/rename_map           std_msgs/String      — JSON {old_name, new_name}
  Pub  /map_editor/rename_map_result    std_msgs/String      — "ok:{new_name}" hoặc "err:{msg}"
  Sub  /map_editor/delete_map           std_msgs/String      — tên map cần xóa
  Pub  /map_editor/delete_map_result    std_msgs/String      — "ok:{name}" hoặc "err:{msg}"

  Pub  /map_editor/keepout_mask     nav_msgs/OccupancyGrid  — filter mask vùng cấm (Nav2 KeepoutFilter)
  Pub  /map_editor/speed_mask       nav_msgs/OccupancyGrid  — filter mask vùng giảm tốc (Nav2 SpeedFilter)
  Pub  /map_editor/filter_info_keepout  nav2_msgs/CostmapFilterInfo
  Pub  /map_editor/filter_info_speed    nav2_msgs/CostmapFilterInfo

  Pub  /localization_status         std_msgs/String   — JSON {status, confidence, cov_x, cov_y}
  Sub  /amcl_pose                   geometry_msgs/PoseWithCovarianceStamped — monitor covariance

Nav2 params cần thêm (xem hướng dẫn cuối file):
  keepout_filter plugin + speed_filter plugin trong global/local costmap

Deploy:
  scp map_editor_node.py pan@10.42.0.229:~/ros2_ws/src/mobile/read_velocity/map_editor_node.py
  pip3 install Pillow   (nếu chưa có)
  python3 ~/ros2_ws/src/mobile/read_velocity/map_editor_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

import json
import base64
import os
import io

try:
    import yaml
except ImportError:
    import subprocess, sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyyaml"])
    import yaml

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    from nav2_msgs.msg import CostmapFilterInfo
    HAS_NAV2_MSGS = True
except ImportError:
    HAS_NAV2_MSGS = False

try:
    from std_srvs.srv import Empty as EmptySrv
    HAS_STD_SRVS = True
except ImportError:
    HAS_STD_SRVS = False

MAPS_DIR        = os.path.expanduser("~/ros2_ws/src/mobile/map")
LAUNCH_FILE     = os.path.expanduser("~/ros2_ws/src/mobile/launch/nav2.launch.py")
THUMB_MAX       = 240    # px — kích thước tối đa chiều dài/rộng của thumbnail
ROBOT_MAX_SPEED = 0.4    # m/s — khớp với max_vel_x / max_speed_xy trong nav2_nguyen.yaml

# Latching QoS: Nav2 nhận được filter info/mask dù node khởi động sau
QOS_LATCH = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class MapEditorNode(Node):
    def __init__(self):
        super().__init__("map_editor_node")

        # ── Standard publishers ───────────────────────────────────────────────
        self.maps_list_pub   = self.create_publisher(String, "/map_editor/maps_list",   10)
        self.image_pub       = self.create_publisher(String, "/map_editor/image",       10)
        self.save_result_pub = self.create_publisher(String, "/map_editor/save_result", 10)

        # ── Thumbnails publisher (latching) ──────────────────────────────────
        self.thumbnails_pub = self.create_publisher(String, "/map_editor/thumbnails", QOS_LATCH)
        self._thumb_cache: dict = {}   # {map_name: b64_png}
        # Build thumbnails 4s sau khi node start để tránh spike lúc khởi động
        self._thumb_init_timer = self.create_timer(4.0, self._initial_thumbnails)

        # ── Nav2 filter mask publishers (latching) ────────────────────────────
        self.keepout_mask_pub = self.create_publisher(OccupancyGrid, "/map_editor/keepout_mask", QOS_LATCH)
        self.speed_mask_pub   = self.create_publisher(OccupancyGrid, "/map_editor/speed_mask",   QOS_LATCH)

        if HAS_NAV2_MSGS:
            self.filter_info_keepout_pub = self.create_publisher(
                CostmapFilterInfo, "/map_editor/filter_info_keepout", QOS_LATCH)
            self.filter_info_speed_pub = self.create_publisher(
                CostmapFilterInfo, "/map_editor/filter_info_speed", QOS_LATCH)
            self._publish_filter_infos()
        else:
            self.get_logger().warning(
                "nav2_msgs không tìm thấy — filter info sẽ không publish. "
                "Cài: sudo apt install ros-$ROS_DISTRO-nav2-msgs"
            )

        # ── Result publishers ─────────────────────────────────────────────────
        self.set_nav_result_pub  = self.create_publisher(String, "/map_editor/set_nav_launch_result", 10)
        self.rename_result_pub   = self.create_publisher(String, "/map_editor/rename_map_result",     10)
        self.delete_result_pub   = self.create_publisher(String, "/map_editor/delete_map_result",     10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(String, "/map_editor/select",         self._on_select,         10)
        self.create_subscription(String, "/map_editor/save",           self._on_save,           10)
        self.create_subscription(String, "/map_editor/set_nav_launch", self._on_set_nav_launch, 10)
        self.create_subscription(String, "/map_editor/rename_map",     self._on_rename_map,     10)
        self.create_subscription(String, "/map_editor/delete_map",     self._on_delete_map,     10)

        self.create_timer(5.0, self._publish_maps_list)
        # Publish filter masks cho map đang active trong nav2.launch.py (chạy 1 lần sau 2s)
        self._auto_mask_timer = self.create_timer(2.0, self._auto_publish_nav_map_masks)

        # ── Localization status (monitor AMCL covariance) ─────────────────────
        self.localization_pub = self.create_publisher(String, "/localization_status", 10)
        self._last_loc_json   = ""
        self._amcl_received   = False
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10
        )
        # Gọi reinitialize_global_localization sau 8s nếu robot chưa định vị
        self._reinit_timer = self.create_timer(8.0, self._trigger_global_localization)

        self.get_logger().info(f"map_editor_node started — maps dir: {MAPS_DIR}")
        if not HAS_PIL:
            self.get_logger().warning("Pillow không tìm thấy — dùng PNG encoder thủ công")

    # ── Thumbnail: chạy 1 lần sau khi node start ─────────────────────────────
    def _initial_thumbnails(self):
        self._thumb_init_timer.cancel()
        self._build_thumbnails()

    # ── Tạo thumbnail nhỏ cho từng map, publish JSON tổng hợp ────────────────
    def _build_thumbnails(self, updated_name: str = None):
        """Tạo/cache thumbnail và publish. updated_name: chỉ rebuild map đó."""
        try:
            names = sorted(
                os.path.splitext(f)[0]
                for f in os.listdir(MAPS_DIR)
                if f.endswith(".yaml") and not f.endswith("_mask.yaml")
            )
            # Xóa cache của map đã bị xóa
            for cached in list(self._thumb_cache.keys()):
                if cached not in names:
                    del self._thumb_cache[cached]

            for name in names:
                if updated_name and name != updated_name and name in self._thumb_cache:
                    continue  # chỉ rebuild map được chỉ định
                pgm_path = os.path.join(MAPS_DIR, f"{name}.pgm")
                if not os.path.exists(pgm_path):
                    continue
                try:
                    if HAS_PIL:
                        img = Image.open(pgm_path).convert("L")
                        w_orig, h_orig = img.size
                        img.thumbnail((THUMB_MAX, THUMB_MAX), Image.LANCZOS)
                        buf = io.BytesIO()
                        img.save(buf, format="PNG", optimize=True)
                        self._thumb_cache[name] = {
                            "thumb":  base64.b64encode(buf.getvalue()).decode("ascii"),
                            "width":  w_orig,
                            "height": h_orig,
                        }
                    else:
                        w_orig, h_orig, _ = _pgm_to_png_manual(pgm_path)
                        self._thumb_cache[name] = {"thumb": "", "width": w_orig, "height": h_orig}
                    self.get_logger().info(f"Thumbnail: '{name}' ({self._thumb_cache[name]['width']}×{self._thumb_cache[name]['height']})")
                except Exception as e:
                    self.get_logger().warning(f"Thumbnail error '{name}': {e}")

            # Đọc resolution từ YAML cho mỗi map
            maps_out = []
            for name in sorted(self._thumb_cache.keys()):
                entry = dict(self._thumb_cache[name])
                entry["name"] = name
                yaml_path = os.path.join(MAPS_DIR, f"{name}.yaml")
                try:
                    with open(yaml_path) as f:
                        meta = yaml.safe_load(f)
                    entry["resolution"] = float(meta.get("resolution", 0.05))
                except Exception:
                    entry["resolution"] = 0.05
                maps_out.append(entry)

            msg = String()
            msg.data = json.dumps({"maps": maps_out, "nav2_map": self._read_nav2_map()})
            self.thumbnails_pub.publish(msg)
            self.get_logger().info(f"Published thumbnails: {len(maps_out)} maps")

        except Exception as e:
            self.get_logger().error(f"_build_thumbnails: {e}")
            import traceback; traceback.print_exc()

    # ── CostmapFilterInfo (static config, publish once với latching QoS) ─────
    def _publish_filter_infos(self):
        now = self.get_clock().now().to_msg()

        info_k = CostmapFilterInfo()
        info_k.header.stamp    = now
        info_k.header.frame_id = "map"
        info_k.type            = 0           # KEEPOUT_FILTER
        info_k.filter_mask_topic = "/map_editor/keepout_mask"
        info_k.base            = 0.0
        info_k.multiplier      = 1.0
        self.filter_info_keepout_pub.publish(info_k)

        info_s = CostmapFilterInfo()
        info_s.header.stamp    = now
        info_s.header.frame_id = "map"
        info_s.type            = 1           # SPEED_FILTER
        info_s.filter_mask_topic = "/map_editor/speed_mask"
        # Nav2 SpeedFilter: nếu speed_limit < 1.0 → hiểu là % của max speed
        # speed_limit = base + multiplier * mask_value
        # mask_value = round(speed_m_s * 100), e.g. 0.1 m/s → 10
        # Muốn: speed_limit = speed_m_s / ROBOT_MAX_SPEED (fraction)
        # → multiplier = 1 / (ROBOT_MAX_SPEED * 100)
        # Ví dụ: 0.1 m/s → mask=10 → speed_limit=0.25 → 25% of 0.4 = 0.1 m/s ✓
        info_s.base            = 0.0
        info_s.multiplier      = 1.0 / (ROBOT_MAX_SPEED * 100)
        self.filter_info_speed_pub.publish(info_s)

        self.get_logger().info("Published CostmapFilterInfo for keepout + speed filters")

    # ── Monitor AMCL covariance → publish /localization_status ───────────────
    def _on_amcl_pose(self, msg):
        self._amcl_received = True
        cov_x   = msg.pose.covariance[0]   # variance X (m²)
        cov_y   = msg.pose.covariance[7]   # variance Y (m²)
        max_cov = max(abs(cov_x), abs(cov_y))

        if max_cov > 0.5:
            status     = "unlocalized"
            confidence = 0.0
        elif max_cov > 0.1:
            confidence = 1.0 - (max_cov - 0.1) / 0.4
            status     = "localizing"
        else:
            confidence = 1.0
            status     = "localized"

        out = json.dumps({
            "status":     status,
            "confidence": round(min(1.0, max(0.0, confidence)), 3),
            "cov_x":      round(cov_x, 4),
            "cov_y":      round(cov_y, 4),
        })
        if out != self._last_loc_json:
            self._last_loc_json = out
            pub_msg = String()
            pub_msg.data = out
            self.localization_pub.publish(pub_msg)

    # ── Gọi reinitialize_global_localization nếu robot chưa định vị ──────────
    def _trigger_global_localization(self):
        self._reinit_timer.cancel()
        if not HAS_STD_SRVS:
            return
        # Bỏ qua nếu robot đã định vị tốt
        if self._last_loc_json:
            try:
                if json.loads(self._last_loc_json).get("status") == "localized":
                    self.get_logger().info(
                        "Robot đã định vị — bỏ qua reinitialize_global_localization"
                    )
                    return
            except Exception:
                pass
        try:
            client = self.create_client(EmptySrv, "/reinitialize_global_localization")
            if client.wait_for_service(timeout_sec=2.0):
                client.call_async(EmptySrv.Request())
                self.get_logger().info(
                    "Gọi /reinitialize_global_localization — AMCL rải particles toàn bản đồ"
                )
            else:
                self.get_logger().warning(
                    "Service /reinitialize_global_localization chưa sẵn sàng — Nav2 chưa bật?"
                )
        except Exception as e:
            self.get_logger().warning(f"_trigger_global_localization: {e}")

    # ── Đọc tên map đang cấu hình trong nav2.launch.py ──────────────────────────
    def _read_nav2_map(self):
        import re as _re
        try:
            if not os.path.exists(LAUNCH_FILE):
                return None
            with open(LAUNCH_FILE) as f:
                content = f.read()
            m = _re.search(
                r"map_file\s*=\s*os\.path\.join\([^,]+,\s*['\"]map['\"]\s*,\s*['\"]([^'\"]+)\.yaml['\"]\)",
                content,
            )
            return m.group(1) if m else None
        except Exception:
            return None

    # ── Auto-publish filter masks khi node start (đọc map từ nav2.launch.py) ──
    def _auto_publish_nav_map_masks(self):
        """Chạy 1 lần 2s sau khi start: tìm map hiện tại trong nav2.launch.py → publish masks.
        Đảm bảo Nav2 luôn nhận đúng keepout/speed mask dù user chưa mở MapEditor."""
        self._auto_mask_timer.cancel()

        import re as _re_auto
        try:
            if not os.path.exists(LAUNCH_FILE):
                self.get_logger().info(
                    "_auto_publish_nav_map_masks: launch file không tồn tại, bỏ qua"
                )
                return

            with open(LAUNCH_FILE) as f:
                content = f.read()

            # Tìm dòng: map_file = os.path.join(pkg_mobile, 'map', '<name>.yaml')
            m = _re_auto.search(
                r"map_file\s*=\s*os\.path\.join\([^,]+,\s*['\"]map['\"]\s*,\s*['\"]([^'\"]+)\.yaml['\"]\)",
                content,
            )
            if not m:
                self.get_logger().info(
                    "_auto_publish_nav_map_masks: không tìm thấy map_file trong launch file"
                )
                return

            map_name  = m.group(1)
            ann_path  = os.path.join(MAPS_DIR, f"{map_name}_annotations.json")
            yaml_path = os.path.join(MAPS_DIR, f"{map_name}.yaml")
            pgm_path  = os.path.join(MAPS_DIR, f"{map_name}.pgm")

            if not os.path.exists(ann_path):
                self.get_logger().info(
                    f"_auto_publish_nav_map_masks: '{map_name}' chưa có annotations, bỏ qua"
                )
                return

            with open(ann_path) as f:
                annotations = json.load(f)
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)

            if HAS_PIL:
                img = Image.open(pgm_path)
                width, height = img.size
            else:
                width, height, _ = _pgm_to_png_manual(pgm_path)

            resolution = float(meta.get("resolution", 0.05))
            origin     = meta.get("origin", [0.0, 0.0, 0.0])

            self._publish_filter_masks(
                annotations, width, height,
                resolution, float(origin[0]), float(origin[1])
            )

            n_ko = sum(1 for a in annotations if a.get("type") == "forbidden")
            n_sp = sum(1 for a in annotations if a.get("type") == "speed")
            self.get_logger().info(
                f"Auto-published filter masks — map: '{map_name}' "
                f"({n_ko} keepout zones, {n_sp} speed zones)"
            )

        except Exception as e:
            self.get_logger().warning(f"_auto_publish_nav_map_masks: {e}")
            import traceback; traceback.print_exc()

    # ── Ghi tên map vào nav2.launch.py để persistent qua restart ─────────────
    def _on_set_nav_launch(self, msg):
        def _reply(text: str):
            r = String(); r.data = text
            self.set_nav_result_pub.publish(r)

        map_name = msg.data.strip()
        if not map_name or "/" in map_name or ".." in map_name:
            _reply(f"err:Tên map không hợp lệ: '{map_name}'"); return

        yaml_path = os.path.join(MAPS_DIR, f"{map_name}.yaml")
        if not os.path.exists(yaml_path):
            _reply(f"err:Map '{map_name}.yaml' không tồn tại trên robot"); return

        if not os.path.exists(LAUNCH_FILE):
            _reply(f"err:Không tìm thấy launch file: {LAUNCH_FILE}"); return

        try:
            with open(LAUNCH_FILE, "r") as f:
                lines = f.readlines()

            new_lines = []
            replaced  = False
            for line in lines:
                stripped = line.lstrip()
                # Tìm dòng map_file = ... không bị comment
                if (not stripped.startswith("#")
                        and "map_file" in stripped
                        and "os.path.join" in stripped
                        and "pkg_mobile" in stripped
                        and "'map'" in stripped):
                    indent = line[: len(line) - len(stripped)]
                    new_lines.append(
                        f"{indent}map_file = os.path.join(pkg_mobile, 'map', '{map_name}.yaml')\n"
                    )
                    replaced = True
                else:
                    new_lines.append(line)

            if not replaced:
                _reply("err:Không tìm thấy dòng map_file trong launch file"); return

            tmp_path = LAUNCH_FILE + ".tmp"
            with open(tmp_path, "w") as f:
                f.writelines(new_lines)
            os.replace(tmp_path, LAUNCH_FILE)

            self.get_logger().info(f"nav2.launch.py updated → map: '{map_name}'")
            _reply(f"ok:{map_name}")

        except Exception as e:
            import traceback; traceback.print_exc()
            _reply(f"err:{e}")

    # ── Đổi tên map: rename .pgm/.yaml/_annotations.json trên robot ─────────
    def _on_rename_map(self, msg):
        import re as _re

        def _reply(text: str):
            r = String(); r.data = text
            self.rename_result_pub.publish(r)

        try:
            payload  = json.loads(msg.data)
            old_name = payload.get("old_name", "").strip()
            new_name = payload.get("new_name", "").strip()

            if not old_name or not new_name:
                _reply("err:Tên không được để trống"); return
            if old_name == new_name:
                _reply("err:Tên mới phải khác tên cũ"); return
            if "/" in new_name or ".." in new_name or " " in new_name:
                _reply(f"err:Tên '{new_name}' chứa ký tự không hợp lệ (/, .., dấu cách)"); return

            old_pgm  = os.path.join(MAPS_DIR, f"{old_name}.pgm")
            old_yaml = os.path.join(MAPS_DIR, f"{old_name}.yaml")
            new_pgm  = os.path.join(MAPS_DIR, f"{new_name}.pgm")
            new_yaml = os.path.join(MAPS_DIR, f"{new_name}.yaml")
            old_ann  = os.path.join(MAPS_DIR, f"{old_name}_annotations.json")
            new_ann  = os.path.join(MAPS_DIR, f"{new_name}_annotations.json")

            if not os.path.exists(old_pgm):
                _reply(f"err:Map '{old_name}.pgm' không tồn tại trên robot"); return
            if os.path.exists(new_pgm) or os.path.exists(new_yaml):
                _reply(f"err:Tên '{new_name}' đã tồn tại trên robot"); return

            # Cập nhật trường image trong YAML (trỏ từ old_name.pgm → new_name.pgm)
            with open(old_yaml) as f:
                yaml_content = f.read()
            yaml_updated = _re.sub(
                r"^(image\s*:\s*)\S+\.pgm",
                f"\\g<1>{new_name}.pgm",
                yaml_content,
                flags=_re.MULTILINE,
            )
            with open(new_yaml, "w") as f:
                f.write(yaml_updated)

            # Rename PGM (sau khi YAML mới đã được ghi thành công)
            os.rename(old_pgm, new_pgm)
            os.remove(old_yaml)

            # Rename annotations nếu có
            if os.path.exists(old_ann):
                os.rename(old_ann, new_ann)

            # Cập nhật thumbnail cache và republish
            if old_name in self._thumb_cache:
                self._thumb_cache[new_name] = self._thumb_cache.pop(old_name)

            self._build_thumbnails(updated_name=new_name)

            self.get_logger().info(f"Renamed map '{old_name}' → '{new_name}'")
            _reply(f"ok:{new_name}")

        except Exception as e:
            import traceback; traceback.print_exc()
            _reply(f"err:{e}")

    # ── Xóa map: xóa .pgm/.yaml/_annotations.json khỏi robot ────────────────
    def _on_delete_map(self, msg):
        def _reply(text: str):
            r = String(); r.data = text
            self.delete_result_pub.publish(r)

        try:
            map_name = msg.data.strip()
            if not map_name or "/" in map_name or ".." in map_name:
                _reply(f"err:Tên map không hợp lệ: '{map_name}'"); return

            pgm_path  = os.path.join(MAPS_DIR, f"{map_name}.pgm")
            yaml_path = os.path.join(MAPS_DIR, f"{map_name}.yaml")
            ann_path  = os.path.join(MAPS_DIR, f"{map_name}_annotations.json")

            if not os.path.exists(pgm_path) and not os.path.exists(yaml_path):
                _reply(f"err:Map '{map_name}' không tồn tại trên robot"); return

            deleted = []
            for path in [pgm_path, yaml_path, ann_path]:
                if os.path.exists(path):
                    os.remove(path)
                    deleted.append(os.path.basename(path))

            # Xóa khỏi thumbnail cache và republish danh sách mới
            self._thumb_cache.pop(map_name, None)
            self._build_thumbnails()

            self.get_logger().info(f"Deleted map '{map_name}': {deleted}")
            _reply(f"ok:{map_name}")

        except Exception as e:
            import traceback; traceback.print_exc()
            _reply(f"err:{e}")

    # ── Publish danh sách map mỗi 5s ─────────────────────────────────────────
    def _publish_maps_list(self):
        try:
            names = sorted(
                os.path.splitext(f)[0]
                for f in os.listdir(MAPS_DIR)
                if f.endswith(".yaml") and not f.endswith("_mask.yaml")
            )
            msg = String()
            msg.data = json.dumps(names)
            self.maps_list_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"_publish_maps_list: {e}")

    # ── Nhận yêu cầu load map → PGM → PNG base64 → publish ──────────────────
    def _on_select(self, msg):
        map_name = msg.data.strip()
        self.get_logger().info(f"Loading map: {map_name}")

        yaml_path = os.path.join(MAPS_DIR, f"{map_name}.yaml")
        pgm_path  = os.path.join(MAPS_DIR, f"{map_name}.pgm")
        ann_path  = os.path.join(MAPS_DIR, f"{map_name}_annotations.json")

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"YAML not found: {yaml_path}"); return
        if not os.path.exists(pgm_path):
            self.get_logger().error(f"PGM not found: {pgm_path}");   return

        try:
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)
            resolution = float(meta.get("resolution", 0.05))
            origin     = meta.get("origin", [0.0, 0.0, 0.0])
            origin_x   = float(origin[0])
            origin_y   = float(origin[1])

            if HAS_PIL:
                img = Image.open(pgm_path)
                width, height = img.size
                buf = io.BytesIO()
                img.save(buf, format="PNG", optimize=True)
                png_bytes = buf.getvalue()
            else:
                width, height, png_bytes = _pgm_to_png_manual(pgm_path)

            b64_str = base64.b64encode(png_bytes).decode("ascii")

            saved_annotations = []
            if os.path.exists(ann_path):
                try:
                    with open(ann_path) as f:
                        saved_annotations = json.load(f)
                    self.get_logger().info(
                        f"Loaded {len(saved_annotations)} annotations from {map_name}_annotations.json"
                    )
                except Exception as ae:
                    self.get_logger().warning(f"Could not read annotations: {ae}")

            out = String()
            out.data = json.dumps({
                "image":       b64_str,
                "width":       width,
                "height":      height,
                "resolution":  resolution,
                "origin_x":    origin_x,
                "origin_y":    origin_y,
                "annotations": saved_annotations,
            })
            self.image_pub.publish(out)

            # Publish filter masks cho Nav2 (latching → Nav2 nhận được khi ready)
            self._publish_filter_masks(
                saved_annotations, width, height, resolution, origin_x, origin_y
            )

            self.get_logger().info(
                f"Published '{map_name}': {width}×{height}, "
                f"PNG {len(png_bytes)//1024}KB, {len(saved_annotations)} annotations"
            )

        except Exception as e:
            self.get_logger().error(f"_on_select error: {e}")
            import traceback; traceback.print_exc()

    # ── Nhận PNG base64 từ web, ghi .pgm + annotations.json về robot ─────────
    def _on_save(self, msg):
        def _reply(text: str):
            r = String(); r.data = text
            self.save_result_pub.publish(r)

        try:
            payload  = json.loads(msg.data)
            map_name = payload.get("name", "").strip()
            b64_img  = payload.get("image", "")

            if not map_name or not b64_img:
                _reply("err:Payload thiếu 'name' hoặc 'image'"); return

            if "/" in map_name or ".." in map_name:
                _reply(f"err:Tên map không hợp lệ: {map_name}"); return

            pgm_path  = os.path.join(MAPS_DIR, f"{map_name}.pgm")
            yaml_path = os.path.join(MAPS_DIR, f"{map_name}.yaml")
            ann_path  = os.path.join(MAPS_DIR, f"{map_name}_annotations.json")

            if not os.path.exists(pgm_path):
                _reply(f"err:Map '{map_name}.pgm' không tồn tại trên robot"); return

            self.get_logger().info(f"Saving map '{map_name}' ({len(b64_img)//1024}KB b64)…")

            # Decode PNG → grayscale → ghi .pgm
            png_bytes = base64.b64decode(b64_img)
            if HAS_PIL:
                img = Image.open(io.BytesIO(png_bytes)).convert("L")
                width, height = img.size
                raw_pixels = img.tobytes()
            else:
                width, height, raw_pixels = _png_to_gray_manual(png_bytes)

            pgm_tmp = pgm_path + ".tmp"
            with open(pgm_tmp, "wb") as f:
                f.write(f"P5\n{width} {height}\n255\n".encode())
                f.write(raw_pixels)
            os.replace(pgm_tmp, pgm_path)
            self.get_logger().info(f"Saved '{map_name}.pgm' ({width}×{height})")

            # Ghi annotations.json
            annotations = payload.get("annotations", [])
            if annotations:
                ann_tmp = ann_path + ".tmp"
                with open(ann_tmp, "w") as f:
                    json.dump(annotations, f)
                os.replace(ann_tmp, ann_path)
                self.get_logger().info(
                    f"Saved {len(annotations)} annotations → '{map_name}_annotations.json'"
                )
            else:
                if os.path.exists(ann_path):
                    os.remove(ann_path)

            # Đọc metadata từ YAML để publish filter masks
            try:
                with open(yaml_path) as f:
                    meta = yaml.safe_load(f)
                resolution = float(meta.get("resolution", 0.05))
                origin     = meta.get("origin", [0.0, 0.0, 0.0])
                self._publish_filter_masks(
                    annotations, width, height,
                    resolution, float(origin[0]), float(origin[1])
                )
            except Exception as me:
                self.get_logger().warning(f"Could not publish filter masks: {me}")

            # Rebuild thumbnail cho map vừa lưu
            try:
                self._build_thumbnails(updated_name=map_name)
            except Exception as te:
                self.get_logger().warning(f"Thumbnail rebuild failed: {te}")

            _reply(f"ok:{map_name}")

        except Exception as e:
            import traceback; traceback.print_exc()
            _reply(f"err:{e}")

    # ── Tạo và publish OccupancyGrid filter masks cho Nav2 ───────────────────
    def _publish_filter_masks(self, annotations, width, height, resolution, origin_x, origin_y):
        now = self.get_clock().now().to_msg()

        forbidden_anns = [a for a in annotations if a.get("type") == "forbidden"]
        speed_anns     = [a for a in annotations if a.get("type") == "speed"]

        def _make_grid(data):
            g = OccupancyGrid()
            g.header.stamp          = now
            g.header.frame_id       = "map"
            g.info.width            = width
            g.info.height           = height
            g.info.resolution       = resolution
            g.info.origin.position.x = origin_x
            g.info.origin.position.y = origin_y
            g.data                  = data
            return g

        # ── Keepout mask: 100 = vùng cấm, 0 = tự do ──────────────────────────
        keepout_data = [0] * (width * height)
        for ann in forbidden_anns:
            pts = ann.get("points", [])
            if len(pts) < 3:
                continue
            for (cx, cy) in _rasterize_polygon(pts, width, height):
                # OccupancyGrid row 0 = bottom (origin_y), canvas row 0 = top → flip y
                oc_row = height - 1 - cy
                if 0 <= oc_row < height and 0 <= cx < width:
                    keepout_data[oc_row * width + cx] = 100

        self.keepout_mask_pub.publish(_make_grid(keepout_data))

        # ── Speed mask: pixel = round(speed_m_s * 100), 0 = không giới hạn ──
        speed_data = [0] * (width * height)
        for ann in speed_anns:
            pts       = ann.get("points", [])
            speed_val = float(ann.get("speedValue", 0.3))
            pv        = min(100, max(1, round(speed_val * 100)))
            if len(pts) < 3:
                continue
            for (cx, cy) in _rasterize_polygon(pts, width, height):
                oc_row = height - 1 - cy
                if 0 <= oc_row < height and 0 <= cx < width:
                    speed_data[oc_row * width + cx] = pv

        self.speed_mask_pub.publish(_make_grid(speed_data))

        self.get_logger().info(
            f"Filter masks published: {len(forbidden_anns)} keepout zones, "
            f"{len(speed_anns)} speed zones"
        )


# ── Rasterize polygon (canvas coords, top-left origin) → list of (col, row) ──
def _rasterize_polygon(points_px, width, height):
    """Scanline fill: trả về danh sách (col, row) pixel bên trong polygon."""
    if len(points_px) < 3:
        return []

    ys  = [p["y"] for p in points_px]
    min_y = max(0, int(min(ys)))
    max_y = min(height - 1, int(max(ys)))
    n   = len(points_px)
    result = []

    for y in range(min_y, max_y + 1):
        intersections = []
        for i in range(n):
            j  = (i + 1) % n
            yi, yj = points_px[i]["y"], points_px[j]["y"]
            xi, xj = points_px[i]["x"], points_px[j]["x"]
            if yi == yj:
                continue
            if min(yi, yj) <= y < max(yi, yj):
                x = xi + (y - yi) * (xj - xi) / (yj - yi)
                intersections.append(x)
        intersections.sort()
        for k in range(0, len(intersections) - 1, 2):
            x0 = max(0, int(intersections[k]))
            x1 = min(width - 1, int(intersections[k + 1]))
            for x in range(x0, x1 + 1):
                result.append((x, y))

    return result


# ── Fallback PNG decoder (không cần Pillow) ───────────────────────────────────
def _png_to_gray_manual(png_bytes: bytes):
    import struct, zlib
    assert png_bytes[:8] == b"\x89PNG\r\n\x1a\n", "Không phải PNG hợp lệ"

    pos = 8; chunks = {}; idat_data = b""
    while pos < len(png_bytes):
        length = struct.unpack(">I", png_bytes[pos:pos+4])[0]
        ctype  = png_bytes[pos+4:pos+8]
        data   = png_bytes[pos+8:pos+8+length]
        pos   += 12 + length
        if   ctype == b"IHDR": w, h, bd, ct = struct.unpack(">IIBB", data[:10]); chunks["IHDR"] = (w, h, bd, ct)
        elif ctype == b"IDAT": idat_data += data
        elif ctype == b"IEND": break

    width, height, _, color_type = chunks["IHDR"]
    raw      = zlib.decompress(idat_data)
    channels = {0: 1, 2: 3, 4: 2, 6: 4}.get(color_type, 1)
    stride   = 1 + width * channels
    out      = bytearray(width * height)

    for y in range(height):
        row = bytearray(raw[y*stride+1: y*stride+1+width*channels])
        if channels == 1:
            out[y*width:(y+1)*width] = row
        else:
            for x in range(width):
                out[y*width+x] = row[x*channels]

    return width, height, bytes(out)


# ── Fallback PNG encoder (không cần Pillow) ───────────────────────────────────
def _pgm_to_png_manual(pgm_path: str):
    import struct, zlib

    with open(pgm_path, "rb") as f:
        assert f.readline().strip() == b"P5", "Chỉ hỗ trợ PGM P5"
        while True:
            line = f.readline()
            if not line.startswith(b"#"): dims = line.decode().strip(); break
        f.readline()  # maxval
        raw = f.read()

    width, height = map(int, dims.split())

    def _chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    scanlines = b"".join(b"\x00" + raw[r*width:(r+1)*width] for r in range(height))
    return width, height, (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(scanlines, 9))
        + _chunk(b"IEND", b"")
    )


def main():
    rclpy.init()
    node = MapEditorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ═══════════════════════════════════════════════════════════════════════════════
# HƯỚNG DẪN CẤU HÌNH NAV2 (thêm vào nav2_params.yaml trên robot)
# ═══════════════════════════════════════════════════════════════════════════════
#
# global_costmap:
#   global_costmap:
#     ros__parameters:
#       plugins: ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter", "speed_filter"]
#       keepout_filter:
#         plugin: "nav2_costmap_2d::KeepoutFilter"
#         enabled: True
#         filter_info_topic: "/map_editor/filter_info_keepout"
#       speed_filter:
#         plugin: "nav2_costmap_2d::SpeedFilter"
#         enabled: True
#         filter_info_topic: "/map_editor/filter_info_speed"
#         speed_limit_topic: "/speed_limit"
#
# local_costmap:
#   local_costmap:
#     ros__parameters:
#       plugins: ["voxel_layer", "inflation_layer", "keepout_filter", "speed_filter"]
#       keepout_filter:
#         plugin: "nav2_costmap_2d::KeepoutFilter"
#         enabled: True
#         filter_info_topic: "/map_editor/filter_info_keepout"
#       speed_filter:
#         plugin: "nav2_costmap_2d::SpeedFilter"
#         enabled: True
#         filter_info_topic: "/map_editor/filter_info_speed"
#         speed_limit_topic: "/speed_limit"
#
# LƯU Ý: "plugins" phải giữ nguyên các plugin cũ + thêm 2 plugin mới ở cuối.
#         Thứ tự plugin ảnh hưởng đến thứ tự xử lý costmap.
# ═══════════════════════════════════════════════════════════════════════════════
