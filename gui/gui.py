import tkinter as tk
from tkinter import scrolledtext, filedialog, simpledialog, PhotoImage
import subprocess
import os
import time
import json

'''
def launch_roslaunch(command):
    global workspace
    # 启动 ROS launch 文件
    subprocess.Popen(
        #f"source /opt/ros/noetic/setup.bash && source {workspace}/devel/setup.bash && {command}",
        f"{command}",
        shell=True,
        executable="/bin/bash"
    )

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

def launch_nav():
    #启动路径规划
    launch_roslaunch("roslaunch sentry_nav sentry_movebase.launch")

def script_pgm():
    global workspace
    script_cmd = f"{workspace}/scripts/save_pgm.sh"
    launch_bash(script_cmd)

def terminate_nav():
    # 关闭导航
    terminate_roslaunch("sentry_movebase","正在关闭导航...")
'''

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

def launch_bash(command):
    subprocess.Popen(
        f"{command}",
        shell=True,
        executable="/bin/bash"
    )

def launch_script(script):
    # 运行脚本
    subprocess.Popen(f"bash {script}", shell=True)

def launch_new_shell_bash(command):
    # 在新的终端中运行指令
    script_cmd = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && {command}; bash'"
    subprocess.Popen(script_cmd, shell=True)

def launch_new_shell_bash1(command):
    # 在新的终端中运行指令
    script_cmd = f"gnome-terminal -- bash -c 'bash; bash {command}'"
    subprocess.Popen(script_cmd, shell=True)

def launch_new_shell_bash_exit(command):
    # 在新的终端中运行指令,结束后关闭终端
    script_cmd = f"gnome-terminal -- bash -c 'source /opt/ros/noetic/setup.bash && {command}; exit'"
    subprocess.Popen(script_cmd, shell=True)

def close_terminal():
    os.system("pkill gnome-terminal")  # 终止所有gnome-terminal进程
    terminate_roslaunch("rviz","正在关闭...")
    terminate_roslaunch("one-touch","正在关闭...")

def get_scans_map():
    # 获取建图地图
    show_message_auto_close("获取建图地图")
    script_cmd = f"sshpass -p 1 scp agile@{robot_ip}:{ws_fastlio}/src/FAST_LIO_LC-master/FAST-LIO/PCD/scans.pcd ./"
    subprocess.Popen(
        f"{script_cmd}",
        shell=True,
        executable="/bin/bash"
    )
    show_message_auto_close("下载地图结束")

def get_nav_map():
    # 获取导航地图
    show_message_auto_close("获取导航地图")
    script_cmd = f"sshpass -p 1 scp agile@{robot_ip}:{ws_fastlio}/src/FAST_LIO_LC-master/PGO/pcd/update_map.pcd ./"
    subprocess.Popen(
        f"{script_cmd}",
        shell=True,
        executable="/bin/bash"
    )
    time.sleep(2)
    show_message_auto_close("下载地图结束")

def get_new_map():
    # 获取最新地图
    launch_new_shell_bash("./test1.sh")

def upload_trace():
    # 上传路径
    txt_to_json()
    script_cmd = f"sshpass -p 1 scp ./target.json agile@{robot_ip}:{ws_location}/src/json/"
    subprocess.Popen(
        f"{script_cmd}",
        shell=True,
        executable="/bin/bash"
    )
    script_cmd = f"sshpass -p 1 scp ./target.json agile@{robot_ip}:{ws_control}/src/bunker_control/src/"
    subprocess.Popen(
        f"{script_cmd}",
        shell=True,
        executable="/bin/bash"
    )
    show_message_auto_close("上传结束")
    script_cmd = f"sshpass -p 1 scp ./update_map.pcd agile@{robot_ip}:{ws_location}/src/FAST_LIO/pcd_L/"
    subprocess.Popen(
        f"{script_cmd}",
        shell=True,
        executable="/bin/bash"
    )
    
def txt_to_json():
    input_filename = 'picking_list.txt'
    output_filename = 'target.json'

    points = []

    with open(input_filename, 'r') as file:
        for line in file:
            # 去除行尾的换行符并分割字符串
            x, y, z = line.strip().split(',')
            # 将分割后的字符串转换为浮点数并添加到列表中
            points.append({"x": float(format(float(x),'.7f')), "y": float(format(float(y),'.7f')), "tag": "normal"})

        # 步骤4: 转换为JSON
    json_data = json.dumps(points, indent=4)  # indent用于美化输出

    with open(output_filename, 'w') as json_file:
        json_file.write(json_data)

    print("转换完成，已保存到", output_filename)


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
ws_fastlio = get_config_value("config","ws_fastlio")
ws_location = get_config_value("config","ws_location")
ws_livox = get_config_value("config","ws_livox")
ws_control = get_config_value("config","ws_control")
ws_bunker = get_config_value("config","ws_bunker")
robot_ip = get_config_value("config","robot_ip")
# 注意:  config 文件是提供相对路径，还是绝对路径。 管理到 启动 python 程序时的路径。 
# 这里：  cd /home/hms/ws_robot 路径。



# 初始化状态
is_started = False

# 创建 GUI
root = tk.Tk()
root.title("ROS Launch Manager")
root.geometry("1000x800")

icon = PhotoImage(file='icon_64x64.png')  
root.tk.call('wm', 'iconphoto', root._w, icon)


label_font = ('Helvetica', 20, 'bold')
button_font = ('Helvetica', 16)

tk.Label(root, text="GUI：", font=label_font).grid(row=0, columnspan=2, pady=(80, 20))

# 使用 grid 平行放置按钮
tk.Button(root, text="スキャンマップのダウンロード", command=get_scans_map, font=button_font, width=35).grid(row=1, column=0, pady=15, padx=5)
tk.Button(root, text="ナビゲーションマップのダウンロード", command=get_nav_map, font=button_font, width=35).grid(row=2, column=0, pady=15, padx=5)

tk.Button(root, text="ナビゲーションパスのアップロード", command=upload_trace, font=button_font, width=35).grid(row=3, column=0, pady=15, padx=5)
#tk.Button(root, text="抓取最新地图", command=get_new_map, font=button_font, width=20).grid(row=2, column=1, pady=15, padx=5)

#tk.Button(root, text="自律走行開始", command=launch_location, font=button_font, width=20).grid(row=3, column=0, pady=15, padx=5)
#tk.Button(root, text="自律走行終了", command=terminate_location, font=button_font, width=20).grid(row=3, column=1, pady=15, padx=5)

#tk.Button(root, text="启动路径规划", command=launch_nav, font=button_font, width=20).grid(row=3, column=0, pady=15, padx=5)
#tk.Button(root, text="停止路径规划", command=terminate_nav, font=button_font, width=20).grid(row=3, column=1, pady=15, padx=5)

#tk.Button(root, text="启动导航", command=launch_nav_start, font=button_font, width=20).grid(row=4, column=0, pady=15, padx=5)
#start_stop_button = tk.Button(root, text="启动", command=launch_start_stop, font=button_font, width=20)
#start_stop_button.grid(row=4, column=1, pady=15, padx=5)

#tk.Button(root, text="备份地图", command=select_and_backup_map, font=button_font, width=20).grid(row=5, column=0, pady=15, padx=5)  
#tk.Button(root, text="恢复地图", command=select_and_restore_map, font=button_font, width=20).grid(row=5, column=1, pady=15, padx=5)  

#tk.Button(root, text="can driver", command=launch_bunker, font=button_font, width=20).grid(row=7, column=0, pady=15, padx=5, columnspan=2)
tk.Button(root, text="閉じる", command=close_terminal, font=button_font, width=35).grid(row=8, column=0, pady=15, padx=5, columnspan=2)

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
