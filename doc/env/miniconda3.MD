按照以下步骤在 Ubuntu 20.04 上安装 Miniconda3（推荐用于轻量化 Python 环境管理）：

✅ 步骤 1：下载安装脚本
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

✅ 步骤 2：运行安装程序
bash Miniconda3-latest-Linux-x86_64.sh

按照提示操作：
	•	输入 yes 接受协议
	•	选择安装路径（默认 ~/miniconda3）
	•	安装完成后建议选 yes 添加到 .bashrc

✅ 步骤 3：使 conda 生效
source ~/.bashrc

如果没有添加，也可以手动激活：
source ~/miniconda3/etc/profile.d/conda.sh

✅ 步骤 4：测试 conda 是否安装成功
conda --version