# 创建 logs 文件夹（如果不存在）
mkdir -p logs

# 启动相机程序，并将日志输出到 logs/camera_log.txt
echo "Starting camera program..."
nohup python3 robot_camera.py > logs/camera_log.txt 2>&1 &
echo "Camera program started. Logs can be found in logs/camera_log.txt."

# 检查端口 5000 是否已被占用
if lsof -i :5000; then
    echo "Port 5000 is already in use. Killing processes occupying the port..."
    # 找到并杀死占用 5000 端口的进程
    fuser -k 5000/tcp
    echo "Processes killed."
else
    echo "Port 5000 is free."
fi

# 进入 server 目录并启动 Gunicorn 服务器
echo "Starting Gunicorn server on port 5000..."
cd server
nohup gunicorn -w 12 -b 0.0.0.0:5000 -k gevent --timeout 0 --worker-connections 2 'monitor:app' > ../logs/cam_server.txt 2>&1 &
echo "Gunicorn server started on port 5000. Logs can be found in ../logs/cam_server.txt."
