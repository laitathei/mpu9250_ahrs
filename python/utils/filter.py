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

def butterworth_coefficient(w, n, mode="low-pass"):
    """
    Calculate butterworth filter coefficient according to the desired order

    :param float w: cut-off frequency in rad/s
    :param float n: order of butterworth filter

    :returns: 
        - a (ndarray) - butterworth filter coefficient
        - b (ndarray) - butterworth filter coefficient with cut-off frequency
        
    .. Reference
    .. [1] 'Low pass filter explanations <https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/Analytic%20Derivation%20for%20Low-Pass%202.0.ipynb>'
    .. [2] 'High pass filter explanations <https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/HighPass/Analytic%20Derivation%20for%20High%20Pass.ipynb>'
    """
    a = np.zeros(n+1)
    a[0] = 1
    gamma = np.pi/(2.0*n)
    for k in range(0,n):
        a[k+1] = (np.cos(k*gamma)/np.sin((k+1)*gamma))*a[k]

    # Calculate butterworth coefficient with cut-off frequency
    c = np.zeros(n+1)
    # low pass calculation
    if mode=="low-pass":
        for j in range(0,n+1):
            c[j] = a[n-j]*pow(w,n-j)
    # high pass calculation
    elif mode=="high-pass":
        for j in range(0,n+1):
            print(pow(w,j))
            c[j] = a[j]/pow(w,j)
    return a, c

def low_pass_filter_1st(w, fs):
    """
    Create 1st order low pass filter by Bilinear (Tustin) transform
    
    :param float w: cut-off frequency in rad/s
    :param float fs: sampling frequency in hz
    :returns: 
        - X (list) - denominator coefficient (X)
        - Y (list) - numerator coefficient (Y)

    .. Reference
    .. [1] 'Low pass filter explanations <https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/Analytic%20Derivation%20for%20Low-Pass%202.0.ipynb>'
    """
    #                                               w         2*pi*f     Y(s)
    # continuous time in frequency domain H(s) = -------- = ---------- = ----
    #                                             s + w     s + 2*pi*f   X(s)
    #                        2(1-z^-1)
    # bilinear transform s = ----------
    #                        dt(1+z^-1)
    #                                             w            w*dt + w*dt*z^-1        Y(z)
    # discrete time in frequency domain H(z) = -------- = -------------------------- = ----
    #                                           s + w     (w*dt - 2) + (w*dt+2)*z^-1   X(z)
    #
    #                                                 -(w*dt - 2)Y[n-1] + (w*dt)X[n] + (w*dt)X[n-1]
    # general form after inverse z transform = Y[n] = ---------------------------------------------
    #                                                                   w*dt + 2
    dt = 1/fs
    D = (w*dt + 2)
    Y = [(-(w*dt - 2))/D]
    X = [(w*dt)/D, (w*dt)/D]
    return X, Y

def low_pass_filter_2nd(w, fs):
    """
    Create 2nd order low pass filter by Bilinear (Tustin) transform
    
    :param float w: cut-off frequency in rad/s
    :param float fs: sampling frequency in hz
    :returns: 
        - X (list) - denominator coefficient (X)
        - Y (list) - numerator coefficient (Y)

    .. Reference
    .. [1] 'Low pass filter explanations <https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/Analytic%20Derivation%20for%20Low-Pass%202.0.ipynb>'
    """
    #                                               w         2*pi*f     Y(s)
    # continuous time in frequency domain H(s) = -------- = ---------- = ----
    #                                             s + w     s + 2*pi*f   X(s)
    #                        2(1-z^-1)
    # bilinear transform s = ----------
    #                        dt(1+z^-1)
    #                                             w                                w^2*dt^2 + 2*w^2*dt^2*z^-1 + w^2*dt^2*z^-2                            Y(z)
    # discrete time in frequency domain H(z) = -------- = ------------------------------------------------------------------------------------------- = ----
    #                                           s + w     (4 + 2*2^0.5*w*dt + w^2*dt^2) + (-8 + 2*w^2*dt^2)*z^-1 + (4 - 2*2^0.5*w*dt + w^2*dt^2)*z^-2   X(z)
    #
    #                                                 (4 - 2*2^0.5*w*dt + w^2*dt^2)Y[n-2] + (-8 + 2*w^2*dt^2)Y[n-1] + (w^2*dt^2)X[n] + (2*w^2*dt^2)X[n-1] + (w^2*dt^2)X[n-1]
    # general form after inverse z transform = Y[n] = ----------------------------------------------------------------------------------------------------------------------
    #                                                                                         4 + 2*2^0.5*w*dt + w^2*dt^2
    dt = 1/fs
    D = (4 + 2*2^0.5*w*dt + w^2*dt^2)
    Y = [(-8 + 2*w**2*dt**2)/D, (4 - 2*2**0.5*w*dt + w**2*dt**2)/D]
    X = [(w**2*dt**2)/D, (2*w**2*dt**2)/D, (w**2*dt**2)/D]
    return X, Y

def high_pass_filter_1st(w, fs):
    """
    Create 1st order high pass filter by Bilinear (Tustin) transform

    :param float w: cut-off frequency in rad/s
    :param float fs: sampling frequency in hz
    :returns: 
        - X (list) - denominator coefficient (X)
        - Y (list) - numerator coefficient (Y)

    .. Reference
    .. [1] 'High pass filter explanations <https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/HighPass/Analytic%20Derivation%20for%20High%20Pass.ipynb>'
    """
    #                                               s           s
    # continuous time in frequency domain H(s) = -------- = ----------
    #                                             s + w     s + 2*pi*f  
    #                        2(1-z^-1)
    # bilinear transform s = ----------
    #                        dt(1+z^-1)
    #                                             s             2 - 2*z^-1               Y(z)
    # discrete time in frequency domain H(z) = -------- = ---------------------------- = ----
    #                                           s + w     (w*dt + 2) + (w*dt - 2)*z^-1   X(z)
    #
    #                                                 -(w*dt - 2)Y[n-1] + (2)X[n] - (2)X[n-1]
    # general form after inverse z transform = Y[n] = ---------------------------------------
    #                                                                w*dt + 2
    dt = 1/fs
    D = (w*dt + 2)
    Y = [(-(w*dt - 2))/D]
    X = [2/D, -2/D]
    return X, Y

def high_pass_filter_2nd(w, fs):
    """
    Create 2nd order high pass filter by Bilinear (Tustin) transform

    :param float w: cut-off frequency in rad/s
    :param float fs: sampling frequency in hz
    :returns: 
        - X (list) - denominator coefficient (X)
        - Y (list) - numerator coefficient (Y)

    .. Reference
    .. [1] 'High pass filter explanations <https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/HighPass/Analytic%20Derivation%20for%20High%20Pass.ipynb>'
    """
    #                                                       s**2           
    # continuous time in frequency domain H(s) = --------------------------
    #                                            s**2 + (2**0.5)*w*s + w**2
    #                        2(1-z^-1)
    # bilinear transform s = ----------
    #                        dt(1+z^-1)
    #                                             s                                  4 - 8*z^-1 + 4*z^-2                                  Y(z)
    # discrete time in frequency domain H(z) = -------- = ----------------------------------------------------------------------------  = ----
    #                                           s + w     (4+2*2^0.5*dt+dt^2*w^2) + (-8+2*dt^2*w^2)*z^-1 + (4-2*2^0.5*dt+dt^2*w^2)*z^-2   X(z)
    #
    #                                                 (4-2*2^0.5*dt+dt^2*w^2)Y[n-2] + (-8+2*dt^2*w^2)Y[n-1]+ (4)X[n] - (8)X[n-1] + (4)X[n-2]
    # general form after inverse z transform = Y[n] = --------------------------------------------------------------------------------------
    #                                                                              (4+2*2^0.5*dt+dt^2*w^2)
    dt = 1/fs
    D = 4 + 2*(2**0.5)*dt + (dt**2)*(w**2)
    X = [4/D, -8/D, 4/D]
    Y = [(-8 + 2*(dt**2)*(w**2))/D, (4 - 2*(2**0.5)*dt + (dt**2)*(w**2))/D]
    return X, Y
    
def sliding_window(data, window_size):
    """
    Sliding window algorithm [1]_

    :param list data: data list
    :param int window_size: sliding window size

    :returns: 
        - result (list) - smoothed data

    .. Reference
    .. [1] 'algorithm explanations <https://blog.csdn.net/u012611644/article/details/126153999>'
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

