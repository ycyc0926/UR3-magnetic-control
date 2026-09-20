# 机械臂离线测试

这里的测试不应发送机械臂运动命令。统一从项目根目录运行：

```bash
source /home/yc/UR3/ros2_env.sh
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider robot/tests
```

`conftest.py` 只把 `robot/` 和 ROS 包测试目录加入 Python 模块搜索路径。
`fixtures/` 只保留验证历史审核失效逻辑所需的最小 JSON，不依赖本地的大型
`planning_checks/` 数据目录。
