from torch.optim.lr_scheduler import LambdaLR

def get_poly_lr_scheduler(optimizer, max_iter, power=0.9):
    lr_lambda = lambda cur_iter: (1 - cur_iter / max_iter) ** power
    return LambdaLR(optimizer, lr_lambda)
