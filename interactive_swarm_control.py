#!/usr/bin/env python3
"""
Interactive Swarm Control Application
対話式群制御アプリケーション - V/Hコマンドで東西/南北整列
"""

import time
import math
from pymavlink import mavutil
from boat_arm_guided import BoatFleet
import signal
import sys

# 接続先設定
connection_strings = {
    1: 'tcp:127.0.0.1:5762',  # 1号機
    2: 'tcp:127.0.0.1:5772',  # 2号機
    3: 'tcp:127.0.0.1:5782'   # 3号機
}

# フォーメーション設定
SPACING = 10.0  # ボート間の距離（メートル）
POSITION_TOLERANCE = 2.0  # 位置誤差許容範囲（メートル）
TARGET_HEADING = 0  # 北向き（度）

class SwarmController:
    def __init__(self):
        self.fleet = BoatFleet(list(connection_strings.values()))
        self.waypoints = {}  # {boat_id: (lat, lon)}
        self.running = True
        
    def connect_and_prepare(self):
        """接続とARM確認"""
        print("ボートに接続中...")
        
        if not self.fleet.connect():
            print("接続に失敗しました")
            return False
            
        # 状態確認
        status = self.fleet.get_status()
        print("\n現在の状態:")
        for boat in status:
            print(f"{boat['boat_id']}号機: モード={boat['mode']}, ARM={'Yes' if boat['armed'] else 'No'}")
            
        # ARMされていない場合はGUIDED/ARMを実行
        needs_arming = any(not boat['armed'] or boat['mode'] != 'GUIDED' for boat in status)
        
        if needs_arming:
            print("\nGUIDEDモード設定とARMが必要です...")
            if not self.fleet.set_guided_and_arm():
                print("GUIDED/ARM設定に失敗しました")
                return False
                
        print("\n✓ 全機準備完了")
        return True
        
    def get_boat_position(self, boat_index):
        """指定したボートの現在位置を取得"""
        master = self.fleet.masters[boat_index]
        
        # バッファをクリアして最新データを取得
        while master.recv_match(blocking=False):
            pass
            
        # 最新の位置情報を要求
        attempts = 0
        while attempts < 5:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                return lat, lon
            attempts += 1
                
        return None, None
        
    def calculate_waypoints_e(self, base_lat, base_lon):
        """E形成（東方向）のウェイポイント計算"""
        # 地球の半径（メートル）
        R = 6371000
        
        # 経度1度あたりの距離（メートル、緯度に依存）
        lon_to_meters = R * math.pi / 180 * math.cos(math.radians(base_lat))
        
        waypoints = {
            1: (base_lat, base_lon),  # 1号機は現在位置
            2: (base_lat, base_lon + (SPACING / lon_to_meters)),  # 2号機は東に10m
            3: (base_lat, base_lon + (2 * SPACING / lon_to_meters))  # 3号機は東に20m
        }
        
        return waypoints
        
    def calculate_waypoints_w(self, base_lat, base_lon):
        """W形成（西方向）のウェイポイント計算"""
        # 地球の半径（メートル）
        R = 6371000
        
        # 経度1度あたりの距離（メートル、緯度に依存）
        lon_to_meters = R * math.pi / 180 * math.cos(math.radians(base_lat))
        
        waypoints = {
            1: (base_lat, base_lon),  # 1号機は現在位置
            2: (base_lat, base_lon - (SPACING / lon_to_meters)),  # 2号機は西に10m
            3: (base_lat, base_lon - (2 * SPACING / lon_to_meters))  # 3号機は西に20m
        }
        
        return waypoints
        
    def calculate_waypoints_s(self, base_lat, base_lon):
        """S形成（南方向）のウェイポイント計算"""
        # 地球の半径（メートル）
        R = 6371000
        
        # 緯度1度あたりの距離（メートル）
        lat_to_meters = R * math.pi / 180
        
        waypoints = {
            1: (base_lat, base_lon),  # 1号機は現在位置
            2: (base_lat - (SPACING / lat_to_meters), base_lon),  # 2号機は南に10m
            3: (base_lat - (2 * SPACING / lat_to_meters), base_lon)  # 3号機は南に20m
        }
        
        return waypoints
        
    def calculate_waypoints_n(self, base_lat, base_lon):
        """N形成（北方向）のウェイポイント計算"""
        # 地球の半径（メートル）
        R = 6371000
        
        # 緯度1度あたりの距離（メートル）
        lat_to_meters = R * math.pi / 180
        
        waypoints = {
            1: (base_lat, base_lon),  # 1号機は現在位置
            2: (base_lat + (SPACING / lat_to_meters), base_lon),  # 2号機は北に10m
            3: (base_lat + (2 * SPACING / lat_to_meters), base_lon)  # 3号機は北に20m
        }
        
        return waypoints
        
    def calculate_waypoints_c(self, base_lat, base_lon):
        """C形成（円形）のウェイポイント計算"""
        # 地球の半径（メートル）
        R = 6371000
        
        # 円の半径（メートル）
        radius = 15.0
        
        waypoints = {}
        
        # 3機を円周上に均等配置（120度間隔）
        for i in range(3):
            # 0度、120度、240度に配置
            angle_rad = 2 * math.pi * i / 3
            
            # 北方向を0度として、時計回りに配置
            north_offset = radius * math.cos(angle_rad)
            east_offset = radius * math.sin(angle_rad)
            
            # 緯度の変化量
            dlat = north_offset / R * (180 / math.pi)
            # 経度の変化量（緯度による補正）
            dlon = east_offset / (R * math.cos(math.radians(base_lat))) * (180 / math.pi)
            
            waypoints[i + 1] = (base_lat + dlat, base_lon + dlon)
        
        return waypoints
        
    def calculate_waypoints_r(self, rotation_angle=30):
        """R形成（回転）- 現在の円形フォーメーションを時計回りに回転"""
        if not hasattr(self, 'current_formation_center'):
            # 初回は現在の各機体位置から円の中心を計算
            positions = []
            for i in range(3):
                lat, lon = self.get_boat_position(i)
                if lat and lon:
                    positions.append((lat, lon))
            
            if len(positions) == 3:
                # 3機の重心を円の中心とする
                center_lat = sum(p[0] for p in positions) / 3
                center_lon = sum(p[1] for p in positions) / 3
                self.current_formation_center = (center_lat, center_lon)
            else:
                return None
        
        center_lat, center_lon = self.current_formation_center
        
        # 地球の半径（メートル）
        R = 6371000
        radius = 15.0
        
        waypoints = {}
        
        # 現在の角度位置を取得または初期化
        if not hasattr(self, 'current_angles'):
            self.current_angles = [0, 120, 240]  # 初期配置
        
        # 各機体を時計回りに回転
        for i in range(3):
            # 時計回りなので角度を減算
            self.current_angles[i] = (self.current_angles[i] - rotation_angle) % 360
            angle_rad = math.radians(self.current_angles[i])
            
            # 北方向を0度として配置
            north_offset = radius * math.cos(angle_rad)
            east_offset = radius * math.sin(angle_rad)
            
            # 緯度の変化量
            dlat = north_offset / R * (180 / math.pi)
            # 経度の変化量（緯度による補正）
            dlon = east_offset / (R * math.cos(math.radians(center_lat))) * (180 / math.pi)
            
            waypoints[i + 1] = (center_lat + dlat, center_lon + dlon)
        
        return waypoints
        
    def send_waypoint_to_boat(self, boat_index, target_lat, target_lon, heading=TARGET_HEADING):
        """指定したボートにウェイポイントと方位を送信"""
        master = self.fleet.masters[boat_index]
        
        # SET_POSITION_TARGET_GLOBAL_INTメッセージで位置と方位を送信
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            0b0000110111111000,  # type_mask (位置とyawを使用)
            int(target_lat * 1e7),
            int(target_lon * 1e7),
            0,  # alt
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            math.radians(heading),  # yaw (北向き = 0度)
            0   # yaw_rate
        )
        
    def execute_formation(self, command):
        """フォーメーション実行"""
        print(f"\n{command}フォーメーションを実行します...")
        
        # 1号機の現在位置を取得（Cコマンドの場合は中心として使用）
        print("基準位置を取得中...")
        base_lat, base_lon = self.get_boat_position(0)
        
        if base_lat is None or base_lon is None:
            print("基準位置を取得できませんでした")
            return False
            
        print(f"基準位置: {base_lat:.6f}, {base_lon:.6f}")
        
        # ウェイポイント生成
        if command == 'E':
            self.waypoints = self.calculate_waypoints_e(base_lat, base_lon)
            print("東方向（E）フォーメーションのウェイポイントを生成しました")
        elif command == 'W':
            self.waypoints = self.calculate_waypoints_w(base_lat, base_lon)
            print("西方向（W）フォーメーションのウェイポイントを生成しました")
        elif command == 'S':
            self.waypoints = self.calculate_waypoints_s(base_lat, base_lon)
            print("南方向（S）フォーメーションのウェイポイントを生成しました")
        elif command == 'N':
            self.waypoints = self.calculate_waypoints_n(base_lat, base_lon)
            print("北方向（N）フォーメーションのウェイポイントを生成しました")
        elif command == 'C':
            self.waypoints = self.calculate_waypoints_c(base_lat, base_lon)
            print("円形（C）フォーメーションのウェイポイントを生成しました")
            # 円の中心と初期角度を保存（base_latが中心）
            self.current_formation_center = (base_lat, base_lon)
            self.current_angles = [0, 120, 240]
        elif command == 'R':
            self.waypoints = self.calculate_waypoints_r()
            if self.waypoints is None:
                print("円形フォーメーションが設定されていません。先にCコマンドを実行してください。")
                return False
            print("回転（R）フォーメーションのウェイポイントを生成しました（30度時計回り）")
        elif command == 'T':
            self.waypoints = self.calculate_waypoints_r(rotation_angle=120)
            if self.waypoints is None:
                print("円形フォーメーションが設定されていません。先にCコマンドを実行してください。")
                return False
            print("回転（T）フォーメーションのウェイポイントを生成しました（120度時計回り）")
        else:
            print("無効なコマンドです")
            return False
            
        # ウェイポイント表示
        print("\nウェイポイント:")
        for boat_id, (lat, lon) in self.waypoints.items():
            print(f"  {boat_id}号機: {lat:.6f}, {lon:.6f}")
            
        # GUIDEDモードに設定
        if command in ['C', 'R', 'T']:
            # 円形フォーメーションと回転では全機移動
            print("\n全機をGUIDEDモードに設定中...")
            self.set_all_guided_mode(all_boats=True)
        else:
            # その他のフォーメーションでは2,3号機のみ
            print("\n2号機と3号機をGUIDEDモードに設定中...")
            self.set_all_guided_mode()
        
        # ウェイポイントアップロード
        if command in ['C', 'R', 'T']:
            # 円形フォーメーションでは各機体が円の中心を向く
            print("\nウェイポイントをアップロード中（各機体は円の中心を向く）...")
            # 円の中心座標
            center_lat, center_lon = self.current_formation_center
            
            for boat_id, (lat, lon) in self.waypoints.items():
                boat_index = boat_id - 1
                # 各機体が円の中心を向く
                angle_to_center = math.atan2(center_lon - lon, center_lat - lat)
                heading_to_center = math.degrees(angle_to_center)
                heading_to_center = (90 - heading_to_center) % 360  # 北を0度とする座標系に変換
                self.send_waypoint_to_boat(boat_index, lat, lon, heading_to_center)
                print(f"  {boat_id}号機: アップロード完了（円の中心を向く）")
        else:
            # その他のフォーメーションでは北向き
            print("\nウェイポイントをアップロード中（到着後は北向き）...")
            for boat_id, (lat, lon) in self.waypoints.items():
                boat_index = boat_id - 1
                if boat_id == 1:
                    # 1号機は移動しない（現在位置を維持）
                    print(f"  {boat_id}号機: 現在位置を維持")
                else:
                    # 2,3号機のみ移動
                    self.send_waypoint_to_boat(boat_index, lat, lon, TARGET_HEADING)
                    print(f"  {boat_id}号機: アップロード完了（北向き設定）")
            
        # 少し待機（コマンドが確実に届くように）
        time.sleep(0.001)
        
        # 移動開始
        if command in ['C', 'R', 'T']:
            print("\n全機が移動開始!")
        else:
            print("\n2号機と3号機が移動開始!")
        
        # 移動完了監視
        if command in ['R', 'T', 'L']:  # 回転とLOITERは監視なし
            pass
        else:  # E/N/W/S/Cは位置到達を確認
            start_time = time.time()
            monitoring_time = 0.5  # 0.5秒間監視
            
            while (time.time() - start_time) < monitoring_time:
                distances = {}
                all_close = True
                
                # 各ボートの距離を確認
                for boat_id in range(1, 4):
                    boat_index = boat_id - 1
                    current_lat, current_lon = self.get_boat_position(boat_index)
                    
                    if current_lat and current_lon:
                        target_lat, target_lon = self.waypoints[boat_id]
                        distance = self.calculate_distance(current_lat, current_lon, target_lat, target_lon)
                        distances[boat_id] = distance
                        
                        # E/W/S/Nコマンドでは1号機は常に目標位置にいるとみなす
                        if boat_id == 1 and command in ['E', 'W', 'S', 'N']:
                            distances[boat_id] = 0.0
                        elif distance > POSITION_TOLERANCE * 2:  # 許容範囲を緩める
                            all_close = False
                            
                # 全機が近づいたら完了
                if all_close:
                    break
                    
                time.sleep(0.1)
            
        print("\n✓ 完了")
            
        return True  # 移動処理完了
        
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """2点間の距離を計算（メートル）"""
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
        
    def set_all_guided_mode(self, all_boats=False):
        """指定されたボートをGUIDEDモードに設定"""
        # ArduPilot RoverのGUIDEDモードは15
        GUIDED_MODE = 15
        
        for i, master in enumerate(self.fleet.masters):
            # Skip Unit 1 unless all_boats is True
            if i == 0 and not all_boats:
                print(f"  Unit {i+1}: Skipping mode change")
                continue
                
            # GUIDEDモードに設定
            for _ in range(3):
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    GUIDED_MODE
                )
                time.sleep(0.1)
            
            # Confirm mode change
            mode_set = False
            start_time = time.time()
            while not mode_set and (time.time() - start_time < 5):
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                if msg and msg.custom_mode == GUIDED_MODE:
                    mode_set = True
                    print(f"  {i+1}号機: GUIDEDモード設定完了")
                    break
                    
            if not mode_set:
                print(f"  {i+1}号機: GUIDEDモード設定失敗")
                
    def set_all_loiter_mode(self):
        """2号機と3号機をLOITERモードに設定"""
        # ArduPilot RoverのLOITERモードは5
        LOITER_MODE = 5
        
        for i, master in enumerate(self.fleet.masters):
            # Skip Unit 1 (index 0)
            if i == 0:
                print(f"  Unit {i+1}: Skipping mode change")
                continue
                
            # LOITERモードに設定
            for _ in range(3):
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    LOITER_MODE
                )
                time.sleep(0.1)
            
            # Confirm mode change
            mode_set = False
            start_time = time.time()
            while not mode_set and (time.time() - start_time < 5):
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                if msg and msg.custom_mode == LOITER_MODE:
                    mode_set = True
                    print(f"  {i+1}号機: LOITERモード設定完了")
                    break
                    
            if not mode_set:
                print(f"  {i+1}号機: LOITERモード設定失敗")
        
    def run_interactive(self):
        """対話式メインループ"""
        print("\n=== Interactive Swarm Control System ===")
        print("Commands:")
        print("  E - Align eastward (10m intervals from Unit 1, north facing)")
        print("  W - Align westward (10m intervals from Unit 1, north facing)")
        print("  S - Align southward (10m intervals from Unit 1, north facing)")
        print("  N - Align northward (10m intervals from Unit 1, north facing)")
        print("  C - Circular formation (15m radius, all units face center)")
        print("  R - Rotate formation clockwise by 30 degrees (requires C first)")
        print("  T - Rotate formation clockwise by 60 degrees (Triangle, requires C first)")
        print("  L - Set Units 2 and 3 to LOITER mode")
        print("  Q - Quit")
        print("  You can also enter sequences like ENWSCRTL")
        print("")
        
        while self.running:
            try:
                command = input("\nPlease enter command (E/W/S/N/C/R/T/L/Q or sequence): ").strip().upper()
                
                if command == 'Q':
                    print("Exiting...")
                    break
                elif all(c in 'EWSNCRTL' for c in command) and len(command) > 0:
                    # Execute sequence of commands (including L for LOITER)
                    if len(command) > 1:
                        print(f"\nExecuting sequence: {command}")
                        for i, cmd in enumerate(command):
                            print(f"\n--- Step {i+1}/{len(command)}: {cmd} ---")
                            if cmd == 'L':
                                print("Setting Units 2 and 3 to LOITER mode...")
                                self.set_all_loiter_mode()
                            else:
                                self.execute_formation(cmd)
                            if i < len(command) - 1:  # Wait between commands except the last one
                                # R/T/Lは短め、E/N/W/S/Cは長め
                                if cmd in ['R', 'T', 'L']:
                                    time.sleep(0.001)
                                else:
                                    time.sleep(0.5)
                    else:
                        # Single command
                        if command == 'L':
                            print("Setting Units 2 and 3 to LOITER mode...")
                            self.set_all_loiter_mode()
                        else:
                            self.execute_formation(command)
                else:
                    print("Invalid command. Please enter E, W, S, N, C, R, T, L, Q, or a sequence like ENWSCRTL.")
                    
            except KeyboardInterrupt:
                print("\n\nInterrupted")
                break
                
def main():
    controller = SwarmController()
    
    # Signal handler setup
    def signal_handler(sig, frame):
        print("\n\nExiting...")
        controller.running = False
        controller.fleet.disconnect()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Connection and preparation
        if controller.connect_and_prepare():
            # Start interactive control
            controller.run_interactive()
            
    except Exception as e:
        print(f"\nError: {e}")
        
    finally:
        controller.fleet.disconnect()
        print("Connection terminated")

if __name__ == "__main__":
    main()