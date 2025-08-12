import torch

model_path = '/home/gift/RL_AIHandkerchief/src/piper_rl_deploy/models/piper_policy.pt'
model = torch.jit.load(model_path)

# 将模型设置为评估模式
model.eval()

# 根据错误信息，将模拟输入的特征维度从 10 改为 12
dummy_input = torch.randn(1, 12)

# 关闭梯度计算
with torch.no_grad():
    output = model(dummy_input)

# 打印输出的形状
print("模型的输出形状是:", output.shape)