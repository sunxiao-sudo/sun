import subprocess
import threading
import time
from threading import Lock

class Handle(object):
    def __init__(self) -> None:
        self.autoware_process = None  # 用于存储 Autoware 进程的引用
        self.lock = Lock()  # 线程锁，用于保证对 `autoware_process` 的线程安全访问

    def start_autoware(self):
        """启动 Autoware 进程，避免阻塞主线程"""
        with self.lock:
            if self.autoware_process is not None:
                print("Autoware 已经在运行中。")
                return
            print("启动 Autoware...")
            # 启动 Autoware 的脚本
            self.autoware_process = subprocess.Popen(['/home/nvidia/code/kunyi/quick_start3.sh'],
                                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            # 输出启动信息（不阻塞）
            print("Autoware 启动中，请稍候...")

    def stop_autoware(self):
        """通过执行 stop.sh 脚本来停止 Autoware 进程"""
        with self.lock:
            print("正在停止 Autoware...")
            # 执行 stop.sh 脚本来停止 Autoware
            stop_process = subprocess.Popen(['/home/nvidia/code/kunyi/stop.sh'],
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
            stdout, stderr = stop_process.communicate()
            if stop_process.returncode == 0:
                print("Autoware 已停止。")
            else:
                print(f"停止 Autoware 时出错: {stderr.decode()}")

    def run(self):
        """主程序循环，接收用户输入"""
        while True:
            print("\n请输入命令:")
            print("1. startautoware - 启动 Autoware")
            print("2. stopautoware - 停止 Autoware")
            print("3. exit - 退出程序")
            
            cmd = input("请输入命令：").strip()

            if cmd == "startautoware":
                # 启动线程来启动 Autoware
                threading.Thread(target=self.start_autoware, daemon=True).start()
            elif cmd == "stopautoware":
                self.stop_autoware()
            elif cmd == "exit":
                print("程序退出...")
                self.stop_autoware()  # 确保退出前停止 Autoware 进程
                break
            else:
                print("无效的命令，请重新输入。")

if __name__ == "__main__":
    handle = Handle()
    handle.run()
