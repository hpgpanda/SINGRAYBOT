import tkinter as tk
from tkinter import scrolledtext, filedialog, simpledialog
import subprocess
import os
import time

def select_and_restore_map(): 
    global workspace
    default_path = f"{workspace}/scripts/bak"
    # 弹出窗口选择PCD文件  
    filepath = filedialog.askopenfilename(filetypes=[("PCD files", "*.pcd")], initialdir=default_path)  
      
    # 如果用户选择了文件，则显示文件路径  
    if filepath:  
        # 获取不含路径和后缀的文件名  
        filename_without_extension = os.path.splitext(os.path.basename(filepath))[0]  
        
        # 创建新窗口显示文件路径  
        dialog_window = tk.Toplevel(root)  
        dialog_window.title("PCD文件")  
        dialog_window.geometry("300x100")  
          
        tk.Label(dialog_window, text=f"选中的PCD文件：{filename_without_extension}").pack(pady=10)  
          
        # 确定按钮
        def on_ok():
            print("确定按钮被点击")
            script_restore_map(filename_without_extension)
            dialog_window.destroy()
          
        # 取消按钮
        def on_cancel():
            print("取消按钮被点击")
            dialog_window.destroy()  
          
        tk.Button(dialog_window, text="确定", command=on_ok).pack(side=tk.LEFT, padx=10)  
        tk.Button(dialog_window, text="取消", command=on_cancel).pack(side=tk.RIGHT, padx=10)  
		
def select_and_backup_map():
    # UI 界面备份地图
    global workspace
    default_folder = f"{workspace}/scripts/bak"  
    # 直接让用户输入文件名，因为文件夹路径已指定  
    new_filename = simpledialog.askstring("输入文件名", "请输入要备份的地图文件名（不含扩展名）:")  
      
    if new_filename:  
        # 执行bak.sh脚本  
        scripts_backup_map(new_filename)  
        show_message_auto_close(f"正在备份地图：{new_filename}", 2000)  
    else:  
        show_message_auto_close(f"请输入文件名！", 2000)  

def show_message_auto_close(message, duration=2000):
    # 信息提示弹出窗口
    popup = tk.Toplevel()
    popup.title("提示")
    
    # 设置初始尺寸并使得窗口居中
    width, height = 300, 100  # 假设提示框的固定尺寸
    screen_width = popup.winfo_screenwidth()
    screen_height = popup.winfo_screenheight()
    x = (screen_width // 2) - (width // 2)
    y = (screen_height // 2) - (height // 2)
    popup.geometry(f"{width}x{height}+{x}+{y}")
    
    # 创建提示标签
    label = tk.Label(popup, text=message, font=('Helvetica', 14))
    label.pack(padx=20, pady=20)

    # 设置自动关闭
    popup.after(duration, popup.destroy)

def launch_roslaunch(command):
    global workspace
    # 启动 ROS launch 文件
    subprocess.Popen(
        #f"source /opt/ros/noetic/setup.bash && source {workspace}/devel/setup.bash && {command}",
        f"{command}",
        shell=True,
        executable="/bin/bash"
    )

def launch_bash(command):
    subprocess.Popen(
        f"{command}",
        shell=True,
        executable="/bin/bash"
    )

def launch_script(script):
    # 运行脚本
    subprocess.Popen(f"bash {script}", shell=True)

def scripts_backup_map(map):
    # 调用脚本备份地图
    global workspace
    script_cmd = f"{workspace}/scripts/bak.sh {workspace} {map}"
    launch_bash(script_cmd)

def script_restore_map(map):
    # 调用脚本恢复地图
    global workspace
    script_cmd = f"{workspace}/scripts/restore.sh {workspace} {map}"
    launch_script(script_cmd)

def terminate_roslaunch(process_name, message):
    # 使用 pkill 终止特定 ROS launch 进程
    subprocess.Popen(f"pkill -f {process_name}", shell=True)
    show_message_auto_close(message)

def launch_rs_livox():
    #启动雷达驱动
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'roslaunch livox_ros_driver2 msg_MID360.launch'])

def launch_fast_lio():
    #启动fast_lio 建图
    workspace_terminal_cmd1 = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && source {workspace}/devel/setup.bash && roslaunch fast_lio mapping_avia.launch'"
    subprocess.Popen(workspace_terminal_cmd1, shell=True)
    # 等待一段时间后打开第二个终端，输入另一个命令
    time.sleep(3)
    workspace_terminal_cmd2 = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && source {workspace}/devel/setup.bash && roslaunch aloam_velodyne fastlio_ouster64.launch'"
    subprocess.Popen(workspace_terminal_cmd2, shell=True)

def terminate_fast_lio():
    #停止建图
    terminate_roslaunch("mapping_avia","正在关闭建图...")
    terminate_roslaunch("fastlio_ouster64","建图已关闭")

def script_pgm():
    global workspace
    script_cmd = f"{workspace}/scripts/save_pgm.sh"
    launch_bash(script_cmd)

def launch_location():
    location_terminal_cmd = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && source {different_workspace}/devel/setup.bash && roslaunch fast_lio localization_mid360.launch'"
    subprocess.Popen(location_terminal_cmd, shell=True)
    # 等待一段时间后执行其他相关命令
    time.sleep(3)
    location_terminal_cmd2 = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && source {different_workspace}/devel/setup.bash && rosrun fast_lio pose_tran'"
    subprocess.Popen(location_terminal_cmd2, shell=True)
    time.sleep(3)
    location_terminal_cmd3 = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && source {different_workspace}/devel/setup.bash && rosrun fast_lio obs_get'"
    subprocess.Popen(location_terminal_cmd3, shell=True)

def terminate_location():
    #关闭重定位
    terminate_roslaunch("localization_mid360","正在关闭重定位...")
    terminate_roslaunch("pose_tran","正在关闭重定位")
    terminate_roslaunch("obs_get","关闭重定位")

def launch_pcdsave():
    pcdsave_terminal_cmd = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && source {different_workspace}/devel/setup.bash && rosrun fast_lio pcd_save'"
    subprocess.Popen(pcdsave_terminal_cmd, shell=True)

def terminate_pcdsave():
    terminate_roslaunch("pcd_save","正在停止保存地图...")

def launch_nav():
    #启动路径规划
    launch_roslaunch("roslaunch sentry_nav sentry_movebase.launch")

def launch_nav_start():
    global start_stop_button
    global is_started    
    terminate_roslaunch("sentry_movebase","正在停止路径规划")
    #subprocess.Popen(f"pkill -f sentry_movebase", shell=True)
    launch_roslaunch("roslaunch sentry_nav sentry_movebase.launch")
    time.sleep(5)
    launch_roslaunch("rosrun fast_lio_localization draw_nav_goal.py")
    time.sleep(5)
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'rosrun robot_base robot_base'])
    if(start_stop_button != None):
        start_stop_button.config(text="停止")
        show_message_auto_close("启动")
        is_started = True

def launch_start_stop():
    global start_stop_button
    global is_started
    if(start_stop_button != None):
        if is_started:
            # 调用代码2
            start_stop_button.config(text="启动")
            terminate_roslaunch("robot_base","正在停止")
        else:
            # 调用代码1
            start_stop_button.config(text="停止")
            terminate_roslaunch("robot_base","正在启动")
            subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'rosrun robot_base robot_base'])
        is_started = not is_started

def terminate_nav():
    # 关闭导航
    terminate_roslaunch("sentry_movebase","正在关闭导航...")

def close_terminal():
    os.system("pkill gnome-terminal")  # 终止所有gnome-terminal进程
    terminate_roslaunch("rviz","正在关闭...")
    terminate_roslaunch("one-touch","正在关闭...")


def get_config_value(filename, key):  
    """  
    从指定文件中读取并返回给定键的值。  
  
    参数:  
    filename (str): 包含配置的文件名。  
    key (str): 要查找的配置键名（不包括等号）。  
  
    返回:  
    str or None: 如果找到键，则返回其值（去除前后空白字符）；如果没有找到，则返回None。  
    """  
    value = None  # 初始化返回值为None  
    with open(filename, 'r') as file:  
        for line in file:  
            # 去除行首和行尾的空白字符，然后检查是否包含指定的键  
            stripped_line = line.strip()  
            if stripped_line.startswith(key + '='):  
                # 如果找到，则分割字符串并返回等号后面的部分（去除前后空白字符）  
                value = stripped_line.split('=')[1].strip()  
                print(value)
                break  # 找到后退出循环  
    return value  





#workspace = "/home/hms/car5_29"
# 获取系统环境变量 $ws
# 这个方法，在终端中 运行 python one-touch_start.py ， 可以实现。 
# 但是通过 start.desktop 的方式来运行， 则会报错。
# 因为 start.desktop 运行时， 不会运行 .bashrc 。
# workspace = os.getenv('ws')
# workspace = get_config_value("/home/hms/ws_robot/scripts/config","ws")
workspace = get_config_value("config","ws1")
different_workspace = get_config_value("config","ws2")
# 注意:  config 文件是提供相对路径，还是绝对路径。 管理到 启动 python 程序时的路径。 
# 这里：  cd /home/hms/ws_robot 路径。


#launch_rs_livox()
# 初始化状态
is_started = False

# 创建 GUI
root = tk.Tk()
root.title("ROS Launch Manager")
root.geometry("600x800")

label_font = ('Helvetica', 20, 'bold')
button_font = ('Helvetica', 16)

tk.Label(root, text="请点击按钮：", font=label_font).grid(row=0, columnspan=2, pady=(80, 20))

# 使用 grid 平行放置按钮
tk.Button(root, text="启动建图", command=launch_fast_lio, font=button_font, width=20).grid(row=1, column=0, pady=15, padx=5)
tk.Button(root, text="停止建图", command=terminate_fast_lio, font=button_font, width=20).grid(row=1, column=1, pady=15, padx=5)

tk.Button(root, text="启动重定位", command=launch_location, font=button_font, width=20).grid(row=2, column=0, pady=15, padx=5)
tk.Button(root, text="停止重定位", command=terminate_location, font=button_font, width=20).grid(row=2, column=1, pady=15, padx=5)

tk.Button(root, text="保存离线地图", command=launch_pcdsave, font=button_font, width=20).grid(row=3, column=0, pady=15, padx=5)
tk.Button(root, text="停止保存地图", command=terminate_pcdsave, font=button_font, width=20).grid(row=3, column=1, pady=15, padx=5)

#tk.Button(root, text="启动路径规划", command=launch_nav, font=button_font, width=20).grid(row=3, column=0, pady=15, padx=5)
#tk.Button(root, text="停止路径规划", command=terminate_nav, font=button_font, width=20).grid(row=3, column=1, pady=15, padx=5)

#tk.Button(root, text="启动导航", command=launch_nav_start, font=button_font, width=20).grid(row=4, column=0, pady=15, padx=5)
#start_stop_button = tk.Button(root, text="启动", command=launch_start_stop, font=button_font, width=20)
#start_stop_button.grid(row=4, column=1, pady=15, padx=5)

#tk.Button(root, text="备份地图", command=select_and_backup_map, font=button_font, width=20).grid(row=5, column=0, pady=15, padx=5)  
#tk.Button(root, text="恢复地图", command=select_and_restore_map, font=button_font, width=20).grid(row=5, column=1, pady=15, padx=5)  

#tk.Button(root, text="保存2D地图", command=script_pgm, font=button_font, width=20).grid(row=7, column=0, pady=15, padx=5, columnspan=2)
tk.Button(root, text="关闭所有终端", command=close_terminal, font=button_font, width=20).grid(row=8, column=0, pady=15, padx=5, columnspan=2)

# 新增：用于显示PCD文件名的文本区域  
#pcd_text_area = scrolledtext.ScrolledText(root, width=30, height=4, state=tk.DISABLED)  
#pcd_text_area.grid(row=5, column=1, columnspan=2, pady=15, padx=5)

# 新增：显示PCD文件名的按钮  
#tk.Button(root, text="显示PCD文件", command=lambda: display_pcd_files("/home/hms"), font=button_font, width=20).grid(row=5, column=0, pady=15, padx=5)  
  
# 创建一个Listbox控件，用于展示PCD文件列表  
#pcd_listbox = tk.Listbox(root)  
#pcd_listbox.grid(row=5, column=1, columnspan=2, pady=15, padx=5, sticky='ew')    
  
# 为Listbox控件绑定双击事件    
#pcd_listbox.bind('<Double-1>', on_file_select)  


root.mainloop()
