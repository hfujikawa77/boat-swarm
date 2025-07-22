#!/usr/bin/env python3
"""
ArduPilot Boat ARM and GUIDED Mode Module
3機のボートをGUIDEDモードに設定してARMする再利用可能なモジュール
"""

import time
from pymavlink import mavutil

class BoatFleet:
    """複数のArduPilotボートを管理するクラス"""
    
    def __init__(self, connection_strings):
        """
        Args:
            connection_strings (list): 各ボートへの接続文字列のリスト
        """
        self.connection_strings = connection_strings
        self.masters = []
        self.connected = False
        
    def connect(self):
        """全てのボートに接続"""
        print("ボートに接続中...")
        
        for i, conn_str in enumerate(self.connection_strings):
            try:
                master = mavutil.mavlink_connection(conn_str)
                master.wait_heartbeat()
                self.masters.append(master)
                print(f"ボート {i+1}: 接続成功 - {conn_str}")
            except Exception as e:
                print(f"ボート {i+1}: 接続失敗 - {conn_str}: {e}")
                self.disconnect()
                return False
                
        self.connected = True
        print(f"\n全{len(self.masters)}機のボートに接続しました。")
        return True
        
    def disconnect(self):
        """全てのボートから切断"""
        for master in self.masters:
            master.close()
        self.masters = []
        self.connected = False
        print("全てのボートから切断しました。")
        
    def get_mode_name(self, mode_num):
        """モード番号から名前を取得"""
        mode_names = {
            0: "MANUAL",
            1: "CIRCLE", 
            2: "STABILIZE",
            3: "TRAINING",
            4: "ACRO",
            5: "FLY_BY_WIRE_A",
            6: "RTL",
            7: "AUTO",
            8: "LAND",
            9: "BRAKE",
            10: "LOITER",
            11: "GUIDED",
            12: "INITIALISING",
            15: "GUIDED",  # Rover/Boatの場合
            16: "HOLD"
        }
        return mode_names.get(mode_num, f"Mode {mode_num}")
        
    def disarm_all(self):
        """全てのボートをDISARM"""
        if not self.connected:
            print("エラー: ボートに接続されていません")
            return False
            
        success = True
        for i, master in enumerate(self.masters):
            print(f"ボート {i+1}: DISARM中...")
            
            # DISARMコマンド送信（複数回）
            for _ in range(3):
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
                
            # DISARM確認
            disarmed = False
            start_time = time.time()
            while not disarmed and (time.time() - start_time < 5):
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    disarmed = True
                    print(f"ボート {i+1}: DISARM完了")
                    
            if not disarmed:
                print(f"ボート {i+1}: DISARM失敗")
                success = False
                
        return success
        
    def set_guided_and_arm(self):
        """全てのボートをGUIDEDモードに設定してARM"""
        if not self.connected:
            print("エラー: ボートに接続されていません")
            return False
            
        print("\n全てのボートをGUIDEDモードに設定してARMします...")
        
        # ArduPilot Rover/BoatのGUIDEDモード
        GUIDED_MODE = 15
        
        all_success = True
        
        for i, master in enumerate(self.masters):
            print(f"\n--- ボート {i+1} ---")
            
            # 現在の状態を確認
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                current_mode = msg.custom_mode
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                print(f"現在のモード: {self.get_mode_name(current_mode)} ({current_mode})")
                print(f"ARM状態: {'ARMED' if is_armed else 'DISARMED'}")
                
                # すでにARMされている場合は一度DISARM
                if is_armed:
                    print("DISARM中...")
                    for _ in range(3):
                        master.mav.command_long_send(
                            master.target_system,
                            master.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 0, 0, 0, 0, 0, 0, 0
                        )
                        time.sleep(0.1)
                    time.sleep(2)
                    
            # GUIDEDモードに設定
            print("GUIDEDモードに設定中...")
            
            # モード変更コマンドを複数回送信
            for _ in range(5):
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    GUIDED_MODE
                )
                time.sleep(0.1)
                
            # モード変更確認
            mode_set = False
            start_time = time.time()
            while not mode_set and (time.time() - start_time < 10):
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                if msg and msg.custom_mode == GUIDED_MODE:
                    mode_set = True
                    print("GUIDEDモード設定完了")
                    
            if not mode_set:
                print("GUIDEDモード設定失敗")
                all_success = False
                continue
                
            # ARM
            print("ARM中...")
            
            # ARMコマンドを複数回送信
            for _ in range(5):
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
                
            # ARM確認
            armed = False
            start_time = time.time()
            while not armed and (time.time() - start_time < 10):
                msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
                if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    armed = True
                    print("ARM完了")
                    
            if not armed:
                print("ARM失敗")
                all_success = False
                
        if all_success:
            print("\n✓ 全てのボートがGUIDEDモードでARMされました。")
        else:
            print("\n✗ 一部のボートでエラーが発生しました。")
            
        return all_success
        
    def get_status(self):
        """全てのボートの状態を取得"""
        if not self.connected:
            return None
            
        status = []
        for i, master in enumerate(self.masters):
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg:
                status.append({
                    'boat_id': i + 1,
                    'mode': self.get_mode_name(msg.custom_mode),
                    'mode_num': msg.custom_mode,
                    'armed': bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                })
            else:
                status.append({
                    'boat_id': i + 1,
                    'mode': 'Unknown',
                    'mode_num': -1,
                    'armed': False
                })
                
        return status


# 使用例
def main():
    """メイン関数（使用例）"""
    connection_strings = [
        'tcp:127.0.0.1:5762',
        'tcp:127.0.0.1:5772',
        'tcp:127.0.0.1:5782'
    ]
    
    # BoatFleetインスタンスを作成
    fleet = BoatFleet(connection_strings)
    
    try:
        # 接続
        if not fleet.connect():
            return
            
        # GUIDEDモード設定とARM
        if fleet.set_guided_and_arm():
            print("\n成功: 全てのボートの準備が完了しました。")
            
            # 状態確認
            print("\n現在の状態:")
            status = fleet.get_status()
            for boat_status in status:
                print(f"ボート {boat_status['boat_id']}: "
                      f"モード={boat_status['mode']}, "
                      f"ARM={'Yes' if boat_status['armed'] else 'No'}")
                      
    except Exception as e:
        print(f"エラー: {e}")
        
    finally:
        # 切断
        fleet.disconnect()


if __name__ == "__main__":
    main()