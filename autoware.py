import subprocess
import threading
import atexit

class Handle(object):
    def __init__(self) -> None:
        self.autoware_process = None  # 用于存储 Autoware 进程的引用
        # 注册退出时的处理函数，确保退出时停止 Autoware 进程
        atexit.register(self.stop_autoware)

    def start_autoware(self):
        # 启动 Autoware，确保没有已有进程在运行
        if self.autoware_process and self.autoware_process.poll() is None:
            print("Autoware is already running.")
            return
        # 启动 Autoware 进程
        self.autoware_process = subprocess.Popen(['/home/nvidia/code/kunyi/quick_start3.sh'],
                                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        print("Autoware started...")

    def stop_autoware(self):
        if self.autoware_process:
            print("Stopping Autoware...")
            self.autoware_process.terminate()  # 优雅停止进程
            self.autoware_process.wait()  # 等待进程退出
            print("Autoware stopped.")
        else:
            print("No Autoware process is running.")

    def start_autoware_in_thread(self):
        # 创建并启动一个新线程来运行 start_autoware
        threading.Thread(target=self.start_autoware).start()

if __name__ == "__main__":
    handle = Handle()

    try:
        # 用户可以选择启动或停止 Autoware
        action = input("Enter 'start' to start Autoware or 'stop' to stop Autoware: ").strip().lower()

        if action == 'start':
            # 启动 Autoware 进程到一个新线程
            handle.start_autoware_in_thread()
        elif action == 'stop':
            handle.stop_autoware()
        else:
            print("Invalid input. Please enter 'start' or 'stop'.")
    finally:
        # 程序退出时，确保调用 stop_autoware 来停止 Autoware 进程
        print("Program is exiting...")
