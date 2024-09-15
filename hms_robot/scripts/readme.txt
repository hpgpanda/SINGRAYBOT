#readme
1、增加一个 上传导航地图的 按键。  
	要实现的功能 就是： 
	将文件 {ws_fastlio}/src/FAST_LIO_LC-master/PGO/pcd/update_map.pcd 通过 scp 复制到 {master_ip}:~/ 目录下。  用户名 hms , 密码 123456. 
	同时将 文件 复制到 {ws_location}/src/FAST_LIO/pcd_L/ 下
	其中{ws_fastlio} 和 {ws_location}， {master_ip}是全局变量。 

2、增加一个 上传稠密地图的 按键。 
	要实现的功能就是：
	打开 路径 {ws_location}/src/FAST_LIO/PCD ， 然客户自己选择要上传的稠密地图。  
	选中后， 通过 scp 上传到 {master_ip}:~/ 目录下。 
	{master_ip}是全局变量。 

3、增加一个 删除稠密地图的 按键。 
	要实现的功能就是：
	打开 路径 {ws_location}/src/FAST_LIO/PCD ， 然客户自己选择要删除的稠密地图。  
	选中后， 将其删除。 
	{master_ip}是全局变量。 

4、增加一个 上传路径的按键
	要实现的功能: 
	通过 scp 将 {master_ip}:~/picking_list.txt 复制到本机 {ws_location}/src/json/ 下。
	将 picking_list.txt 文件内的 X和 Y 值 填写到 {ws_location}/src/json/target.json 文件中。
	并将 {ws_location}/src/json/target.json 复制到 {ws_control}/src/bunker_control/src/target.json