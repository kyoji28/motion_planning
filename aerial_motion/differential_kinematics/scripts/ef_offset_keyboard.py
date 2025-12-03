#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import termios
import tty

class EfOffsetKeyboard:
    def __init__(self):
        # このノード自身
        rospy.init_node('ef_offset_keyboard')

        # 対象ノードの名前空間（デフォルトは、/pose_transformer）
        self.ns = rospy.get_param('~target_ns', '/pose_transformer')

        # 今どのパラメータを編集しているか
        # 'x', 'y', 'z', 'roll', 'pitch', 'yaw' のいずれか
        self.control_target = 'x'

        # 増減ステップ
        self.step_pos = 0.01 # m 位置オフセット
        self.step_angle = 0.05 # rad 角度オフセット
        
    # ---- ヘルパ ----
    def _full_name(self, name: str) -> str:
        """名前空間付きのパラメータ名を返す"""
        return self.ns.rstrip('/') + '/' + name

    def _get_param(self, name: str, default: float = 0.0) -> float:
        """対象ノードのパラメータを取得する"""
        full = self._full_name(name)
        return rospy.get_param(full, default)
        
    def _set_param(self, name: str, value: float):
        """対象ノードのパラメータを設定"""
        full = self._full_name(name)
        rospy.set_param(full, value)
        rospy.loginfo("set %s = %.6f", full, value)

    # ---- 6軸一括入力モード ----
    def input_all_once(self):
        print("x, y, z, roll, pitch, yaw をスペース区切りで入力してください")
        print("例: 0.1 0.0 0.2 0.0 0.1 3.14")

        line = input("> ").strip()
        if not line:
            print("入力が空です。")
            return  
    
        parts = line.split()
        if len(parts) != 6:
            print("6つの値を入力してください（入力個数： {}）".format(len(parts)))
            return

        try:
            vals = [float(x) for x in parts]
        except ValueError:
            print("数値に変換できませんでした。もう一度お試しください。")
            return 
    
        names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        for name, val in zip (names, vals):
            self._set_param(name, val)

        print("6軸オフセットを更新しました。")

    # ---- モード切り替え＋増減 ----
    def change_mode_or_increment(self, cmd: str):
        """
        cmdに応じてモード変更 or 増減を行う。
        cmd: 'x', 'y', 'z', 'r', 'p', 'u', 'w', 's'など
        """

        # --- まずは「モード切り替え」かどうかを見る ---
        if cmd in ['x', 'y', 'z', 'r', 'p', 'u']:
            # 編集対象の切り替え
            if cmd == 'x':
                self.control_target = 'x'
                print("\n--- 編集対象を x に変更しました ---")
            elif cmd == 'y':
                self.control_target = 'y'
                print("\n--- 編集対象を y に変更しました ---")
            elif cmd == 'z':
                self.control_target = 'z'
                print("\n--- 編集対象を z に変更しました ---")
            elif cmd == 'r':
                self.control_target = 'roll'
                print("\n--- 編集対象を roll に変更しました ---")
            elif cmd == 'p':
                self.control_target = 'pitch'
                print("\n--- 編集対象を pitch に変更しました ---")
            elif cmd == 'u':
                self.control_target = 'yaw'
                print("\n--- 編集対象を yaw に変更しました ---")

            self.show_current_offsets(short=False)
            return # ここで終了（増減処理には進まない）

        # --- 次に、「増減」コマンドかどうかを見る ---
        if cmd in ['w', 's']:
            direction = +1 if cmd == 'w' else -1
            self._increment_target(direction)
        # 現在地を1行で表示
            self.show_current_offsets(short=True)
            return

        print("未知のコマンドです： {}".format(cmd))

        # 状態の表示
        self.show_current_offsets(short=True)
    
    def _increment_target(self, direction: int):
        """現在のcontrol_targetをdirection(+1/-1)方向に増減"""
        name = self.control_target
        
        # 今の値を取得（なければ0.0）
        current = self._get_param(name, 0.0)

        " 位置 or 角度 でステップを変える"
        if name in ['x', 'y', 'z']:
            step = self.step_pos * direction
        else:
            step = self.step_angle * direction

        new_val = current + step
        self._set_param(name, new_val)
    
    def continuous_mode(self):
        """
        1文字ずつ即反応する「連続操作モード」
        x/y/z/r/p/u : 編集対象を変更
        w/s : 現在の編集対象を増減
        m : 現在のオフセット値を表示
        b : メニューに戻る
        q : ノード終了
        """

        print("\n=== 連続操作モード開始 ===")
        print("x/y/z/r/p/u : 編集対象を変更")
        print("w : 現在の編集対象を　+ 方向に増加")
        print("s : 現在の編集対象を　- 方向に減少")
        print("m : 現在のオフセット値を表示")
        print("b : メニューに戻る")
        print("q : 終了") 
        print("=========================\n")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            # キーボード入力を「1文字即時読み取り」モードにする
            tty.setcbreak(fd)

            while not rospy.is_shutdown():
                ch = sys.stdin.read(1)  # 1文字読み取り

                if ch == 'b':
                    print("メニューに戻ります。")
                    break
                elif ch == 'q':
                    print("終了します。")
                    rospy.signal_shutdown("ユーザによる終了要求")
                    break
                elif ch in ['x', 'y', 'z', 'r', 'p', 'u', 'w', 's']:
                    self.change_mode_or_increment(ch)
                elif ch == 'm':
                    self.show_current_offsets(short=False)
                else:
                    print("未知のコマンドです： {}".format(ch)) 
        finally:
            # 元の端末設定に戻す
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("=== 連続操作モード終了 ===\n")



    def show_current_offsets(self, short: bool = False):
        """現在のオフセット値を表示"""
        names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        vals = [self._get_param(n, 0.0) for n in names]

        if short:
            print("mode = {} | ".format(self.control_target) + 
            "".join(["{}={:.3f}".format(n,v) for n, v in zip(names, vals)]))
        else:
            print("\n=== 現在のオフセット値 ===")
            for n, v in zip(names, vals):
                mark = "<-" if n == self.control_target else " "
                print("{:>5} = {: .6f} {}".format(n, v, mark))
            print("=========================\n")
    
    def run(self):
        print("=== ef_offset_keyboard ノード起動 ===")
        print("対象ノード（名前空間）： {}".format(self.ns))
        print("pose_transformer側では、hand_pose_cbの中で、~xなどのパラメータを毎回取得して、")
        print("ef_pose_in_handを更新するようにしておいてください。")

        while not rospy.is_shutdown():
            print("\n --- メニュー ---")
            print(" a : x, y, z, roll, pitch, yaw　を一括入力")
            print(" x/y/z/r/p/u : 編集対象を変更")
            print(" w : 現在の編集対象を　+ 方向に増加")
            print(" s : 現在の編集対象を　- 方向に減少")
            print(" m : 現在のオフセット値を表示")
            print(" c : 連続操作モードへ")
            print(" q : 終了")
            cmd = input("コマンドを入力してください：").strip()
            
            if not cmd:
                continue
            
            if cmd == 'q':
                print("終了します。")
                break
            elif cmd == 'a':
                self.input_all_once()
            elif cmd == 'm':
                self.show_current_offsets(short=False)
            elif cmd == 'c':
                self.continuous_mode()
            elif cmd in ['x', 'y', 'z', 'r', 'p', 'u', 'w', 's']:
                self.change_mode_or_increment(cmd)
            else:
                print("未知のコマンドです： {}".format(cmd))

if __name__ == '__main__':
    try:
        node = EfOffsetKeyboard()
        node.run()
    except rospy.ROSInterruptException:
        pass