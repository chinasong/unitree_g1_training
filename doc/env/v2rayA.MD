🛠️ 系统要求
	•	系统：Ubuntu 20.04+
	•	权限：需要 root 权限（使用 sudo）

✅ 步骤一：安装 v2rayA

# 安装必要依赖
sudo apt update
sudo apt install -y curl gnupg apt-transport-https ca-certificates

# 导入公钥
curl -fsSL https://apt.v2raya.org/key/public-key.asc | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/v2raya.gpg > /dev/null

# 添加 v2rayA 源
echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/v2raya.gpg] https://apt.v2raya.org/ stable main" | sudo tee /etc/apt/sources.list.d/v2raya.list

# 更新并安装 v2rayA
sudo apt update
sudo apt install -y v2raya

✅ 步骤二：安装 V2Ray 或 Xray 核心（推荐 Xray）
# 安装 Xray
sudo apt install -y xray

# 或者安装原版 V2Ray（不推荐）
# sudo apt install -y v2ray

✅ 步骤三：启动 v2rayA
# 启动服务
sudo systemctl start v2raya

# 设置开机自启
sudo systemctl enable v2raya

✅ 步骤四：访问 v2rayA 图形界面
http://127.0.0.1:2017

✅ 步骤五（可选）：防火墙设置
如果你启用了 UFW 防火墙，请开放端口：
sudo ufw allow 2017/tcp


在G1机器人中arm64 v2ray安装
✅ 一、下载 V2Ray ARM64 架构版本
你可以去 GitHub Releases 下载：

👉 https://github.com/v2fly/v2ray-core/releases

找到类似于：
v2ray-linux-arm64.zip

✅ 二、解压文件
cd ~/Downloads
unzip v2ray-linux-arm64.zip -d v2ray-arm64
cd v2ray-arm64

✅ 三、复制到系统路径（可选）
sudo install -m 755 v2ray /usr/local/bin/v2ray
sudo install -m 755 v2ctl /usr/local/bin/v2ctl

✅ 四、配置文件目录
sudo mkdir -p /usr/local/etc/v2ray
sudo cp -r geo* config.json /usr/local/etc/v2ray/

✅ 五、手动启动测试
sudo /usr/local/bin/v2ray -config /usr/local/etc/v2ray/config.json

✅ 六（可选）：创建 systemd 服务（自动启动）
sudo nano /etc/systemd/system/v2ray.service

粘贴以下内容：
[Unit]
Description=V2Ray Service
After=network.target

[Service]
ExecStart=/usr/local/bin/v2ray -config /usr/local/etc/v2ray/config.json
Restart=on-failure

[Install]
WantedBy=multi-user.target

然后运行：
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable v2ray
sudo systemctl start v2ray

✅ 七、验证是否监听成功
netstat -tulnp | grep v2ray

网页可以访问google.com了，怎么配置命令安装这些也可以访问外网
http://localhost:2017

默认配置下：
	•	SOCKS5 代理：127.0.0.1:10808
	•	HTTP 代理：127.0.0.1:10809

你可以在 “设置” 页面 → “代理端口” 查看或修改。

✅ 步骤二：让命令行走代理（临时设置）
export http_proxy=http://127.0.0.1:10809
export https_proxy=http://127.0.0.1:10809

现在测试一下：
curl https://google.com

✅ 可选：永久设置代理（加到 bash 配置里）
编辑你的 .bashrc 或 .zshrc 文件：
nano ~/.bashrc

添加这几行：
export http_proxy=http://127.0.0.1:10809
export https_proxy=http://127.0.0.1:10809

然后执行：
source ~/.bashrc

🔄 如果使用的是 SOCKS5（而不是 HTTP）
需要安装代理工具，如 proxychains 或 tsocks，或者使用：
export ALL_PROXY=socks5h://127.0.0.1:10808

G1机器人临时修复网络外网问题：sudo ip route del default via 192.168.123.1 dev eth0
