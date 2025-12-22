# Ubuntu中配置DEEPSEEK_API_KEY环境变量

## 方法1：临时设置（仅当前终端会话有效）

在终端中直接运行：

```bash
export DEEPSEEK_API_KEY="sk-7d7bdbb2f793433082e88dd094a0765d"
```

**注意**：这种方式只在当前终端窗口有效，关闭终端后就会失效。

验证是否设置成功：
```bash
echo $DEEPSEEK_API_KEY
```

## 方法2：永久设置（用户级别，推荐）

### 2.1 使用 ~/.bashrc（适用于bash shell）

```bash
# 编辑 ~/.bashrc 文件
nano ~/.bashrc
# 或者
vim ~/.bashrc
# 或者
gedit ~/.bashrc
```

在文件末尾添加：
```bash
# DeepSeek API配置
export DEEPSEEK_API_KEY="sk-7d7bdbb2f793433082e88dd094a0765d"
```

保存文件后，运行以下命令使配置生效：
```bash
source ~/.bashrc
```

### 2.2 使用 ~/.profile（适用于所有shell）

```bash
# 编辑 ~/.profile 文件
nano ~/.profile
```

在文件末尾添加：
```bash
# DeepSeek API配置
export DEEPSEEK_API_KEY="sk-7d7bdbb2f793433082e88dd094a0765d"
```

保存文件后，运行：
```bash
source ~/.profile
```

或者重新登录使配置生效。

### 2.3 使用 ~/.bash_profile（如果存在）

```bash
# 编辑 ~/.bash_profile 文件
nano ~/.bash_profile
```

添加相同的export语句，然后：
```bash
source ~/.bash_profile
```

## 方法3：使用环境变量文件（推荐用于项目）

### 3.1 创建 .env 文件（需要安装python-dotenv）

首先安装python-dotenv：
```bash
pip3 install python-dotenv
```

在项目目录（cameradev/）下创建 `.env` 文件：
```bash
cd cameradev
nano .env
```

添加内容：
```
DEEPSEEK_API_KEY=sk-7d7bdbb2f793433082e88dd094a0765d
```

然后在Python代码中加载（需要修改代码以支持.env文件）。

### 3.2 创建独立的配置文件

在 `cameradev/` 目录下创建 `set_env.sh`：
```bash
cd cameradev
nano set_env.sh
```

添加内容：
```bash
#!/bin/bash
export DEEPSEEK_API_KEY="sk-7d7bdbb2f793433082e88dd094a0765d"
```

赋予执行权限：
```bash
chmod +x set_env.sh
```

使用方式：
```bash
source set_env.sh
python3 g1_humorous_chat.py eth0
```

## 方法4：系统级别配置（所有用户）

**注意**：不推荐，除非你确实需要所有用户都能访问。

编辑 `/etc/environment`：
```bash
sudo nano /etc/environment
```

添加：
```
DEEPSEEK_API_KEY="sk-7d7bdbb2f793433082e88dd094a0765d"
```

然后重新登录或重启系统。

## 验证配置

### 检查环境变量是否设置

```bash
# 方法1：使用echo
echo $DEEPSEEK_API_KEY

# 方法2：使用env命令
env | grep DEEPSEEK_API_KEY

# 方法3：使用printenv
printenv DEEPSEEK_API_KEY
```

### 在Python中测试

创建一个测试脚本 `test_env.py`：
```python
import os
api_key = os.getenv("DEEPSEEK_API_KEY")
if api_key:
    print(f"✅ API密钥已设置: {api_key[:10]}...")
else:
    print("❌ API密钥未设置")
```

运行：
```bash
python3 test_env.py
```

## 推荐方案

**对于个人开发使用**：推荐使用方法2.1（~/.bashrc），因为：
- 配置简单
- 永久有效
- 只影响当前用户
- 不需要修改代码

**对于项目部署**：推荐使用方法3.2（set_env.sh），因为：
- 配置与项目绑定
- 便于版本控制（可以添加到.gitignore）
- 不同项目可以使用不同的密钥

## 常见问题

### Q1: 设置了环境变量但程序还是找不到？

**A**: 确保：
1. 使用了 `source ~/.bashrc` 或重新打开终端
2. 在同一个终端会话中运行程序
3. 检查变量名是否正确（区分大小写）

### Q2: 如何取消环境变量？

**A**: 
```bash
# 临时取消（当前会话）
unset DEEPSEEK_API_KEY

# 永久取消：从 ~/.bashrc 或 ~/.profile 中删除export语句
```

### Q3: 如何查看所有环境变量？

**A**:
```bash
env
# 或
printenv
```

### Q4: 在systemd服务中如何使用？

**A**: 在systemd服务文件中添加：
```ini
[Service]
Environment="DEEPSEEK_API_KEY=sk-7d7bdbb2f793433082e88dd094a0765d"
```

## 安全建议

1. **不要将API密钥提交到Git**：确保 `.env` 和包含密钥的脚本在 `.gitignore` 中
2. **使用文件权限保护**：如果使用脚本文件，设置适当的权限：
   ```bash
   chmod 600 set_env.sh  # 只有所有者可读写
   ```
3. **定期轮换密钥**：如果密钥泄露，及时更换

## 快速设置脚本

创建一个快速设置脚本 `setup_env.sh`：

```bash
#!/bin/bash

API_KEY="sk-7d7bdbb2f793433082e88dd094a0765d"

# 检查是否已设置
if grep -q "DEEPSEEK_API_KEY" ~/.bashrc; then
    echo "⚠️  DEEPSEEK_API_KEY 已在 ~/.bashrc 中设置"
    echo "当前值: $(grep DEEPSEEK_API_KEY ~/.bashrc)"
else
    echo "export DEEPSEEK_API_KEY=\"$API_KEY\"" >> ~/.bashrc
    echo "✅ 已添加 DEEPSEEK_API_KEY 到 ~/.bashrc"
    echo "请运行: source ~/.bashrc"
fi
```

使用方法：
```bash
chmod +x setup_env.sh
./setup_env.sh
source ~/.bashrc
```

