import numpy as np

def get_confusion_matrix(label, pred, size, num_class, ignore=-1):
    seg_pred = pred.cpu().numpy()
    seg_gt = np.asarray(label.cpu().numpy()[:, :size[-2], :size[-1]], dtype=int)

    seg_pred = seg_pred[seg_gt != ignore]
    seg_gt = seg_gt[seg_gt != ignore]

    index = (seg_gt * num_class + seg_pred).astype('int32')
    label_count = np.bincount(index)
    confusion_matrix = np.zeros((num_class, num_class))

    for i_label in range(num_class):
        for i_pred in range(num_class):
            cur_index = i_label * num_class + i_pred
            if cur_index < len(label_count):
                confusion_matrix[i_label,
                                 i_pred] = label_count[cur_index]
                
    return confusion_matrix