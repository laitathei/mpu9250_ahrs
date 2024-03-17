import numpy as np

# def butterworth(pf_l, pf_h, sf_l, sf_h, t):
#     """
#     cutoff frequency defined by half-power point (-3dB)
#     pf_l: lower passband frequency
#     pf_h: upper passband frequency
#     sf_l: lower stopband frequency
#     sf_h: upper stopband frequency
#     t: sampling time
#     """
#     # calculate passband bandwidth frequency
#     pf_b = pf_h - pf_l

#     # calculate central frequency
#     cf = pf_l * pf_h

#     # normalize frequency
#     pf_l = pf_l/pf_b
#     pf_h = pf_h/pf_b
#     sf_l = sf_l/pf_b
#     sf_h = sf_h/pf_b

def sliding_window(data, window_size):
    """
    Sliding window algorithm

    :param list data: data list
    :param int window_size: sliding window size

    :returns: 
        - result - smoothed data

    .. Reference
    .. [1] `algorithm explanations <https://blog.csdn.net/u012611644/article/details/126153999>`
    """
    if window_size > len(data):
        raise ValueError("Window size larger than data length")

    result = []
    for i in range(len(data) - window_size + 1):
        # Get the element within current window
        window_element = data[i:i+window_size]

        # # Remove maximum value and minimum value
        # window_element.remove(max(window_element))
        # window_element.remove(min(window_element))
        # # Get filter out result
        # result.append(sum(window_element)/(window_size-2))

        # Get filter out result
        result.append(sum(window_element)/(window_size))

    if isinstance(data, np.ndarray):
        result = np.array(result)

    return result
