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

临时修复网络外网问题：sudo ip route del default via 192.168.123.1 dev eth0