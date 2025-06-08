在 Ubuntu 20.04 上安装 NoMachine（一款强大的远程桌面工具）非常简单。以下是完整的安装步骤：

✅ 方法一：使用官网下载的 .deb 安装包（推荐）

1. 下载 NoMachine .deb 安装包

前往官网：

👉 https://www.nomachine.com/download/linux&id=1

选择：“NoMachine for Linux DEB i386/AMD64”
下载的是类似于 nomachine_8.x.x_x_amd64.deb 的文件。

如果是 ARM64 架构设备（比如 Raspberry Pi 或 ARM 服务器），点击：
👉 https://www.nomachine.com/download/linux&id=7
选择 “DEB arm64” 版本下载。

2. 安装依赖（可选但推荐）
sudo apt update
sudo apt install -y libx11-6

3. 安装 .deb 文件

假设你下载在 ~/Downloads/ 文件夹中：
cd ~/Downloads
sudo dpkg -i nomachine_*.deb

如果报依赖错误，运行：
sudo apt --fix-broken install

✅ 方法二：命令行一键安装（适用于 x86_64）
wget https://download.nomachine.com/download/8.12/Linux/nomachine_8.12.3_1_amd64.deb
sudo dpkg -i nomachine_8.12.3_1_amd64.deb

✅ 安装完成后
	•	默认会启动 NoMachine 服务
	•	本地访问地址：nx://<你的IP地址>
	•	打开 NoMachine 客户端连接即可
	•	配置文件位置在：/usr/NX/etc/

✅ 卸载命令
sudo /usr/NX/scripts/setup/nxserver --uninstall

要在 Ubuntu 中启动 NoMachine 的服务，请按以下步骤操作：

✅ 1. 启动 NoMachine 服务
sudo /usr/NX/bin/nxserver --startup

✅ 2. 查看服务状态
sudo /usr/NX/bin/nxserver --status

输出示例应包括：
NX> 161 Enabled service: nxserver
NX> 162 NX service is running

✅ 3. 设置为开机自动启动（可选）
如果你希望每次开机自动运行 NoMachine 服务：
sudo /usr/NX/bin/nxserver --install

✅ 4. 重启 NoMachine 服务（可选）
sudo /usr/NX/bin/nxserver --restart

一般在G1机器人系统里192.168.123.164进入，运行sudo ./nomachine.sh 即可 

