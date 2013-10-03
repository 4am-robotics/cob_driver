import numpy as np
from math import factorial
import pylab

class savitzky_golay():
    r"""Smooth (and optionally differentiate) data with a Savitzky-Golay filter.
    The Savitzky-Golay filter removes high frequency noise from data.
    It has the advantage of preserving the original shape and
    features of the signal better than other types of filtering
    approaches, such as moving averages techniques.
    Parameters
    ----------
    y : array_like, shape (N,)
        the values of the time history of the signal.
    window_size : int
        the length of the window. Must be an odd integer number.
    order : int
        the order of the polynomial used in the filtering.
        Must be less then `window_size` - 1.
    deriv: int
        the order of the derivative to compute (default = 0 means only smoothing)
    Returns
    -------
    ys : ndarray, shape (N)
        the smoothed signal (or it's n-th derivative).
    Notes
    -----
    The Savitzky-Golay is a type of low-pass filter, particularly
    suited for smoothing noisy data. The main idea behind this
    approach is to make for each point a least-square fit with a
    polynomial of high order over a odd-sized window centered at
    the point.
    Examples
    --------
    t = np.linspace(-4, 4, 500)
    y = np.exp( -t**2 ) + np.random.normal(0, 0.05, t.shape)
    ysg = savitzky_golay(y, window_size=31, order=4)
    import matplotlib.pyplot as plt
    plt.plot(t, y, label='Noisy signal')
    plt.plot(t, np.exp(-t**2), 'k', lw=1.5, label='Original signal')
    plt.plot(t, ysg, 'r', label='Filtered signal')
    plt.legend()
    plt.show()
    References
    ----------
    .. [1] A. Savitzky, M. J. E. Golay, Smoothing and Differentiation of
       Data by Simplified Least Squares Procedures. Analytical
       Chemistry, 1964, 36 (8), pp 1627-1639.
    .. [2] Numerical Recipes 3rd Edition: The Art of Scientific Computing
       W.H. Press, S.A. Teukolsky, W.T. Vetterling, B.P. Flannery
       Cambridge University Press ISBN-13: 9780521880688
    """

    def __init__(self, window_size, order, deriv=0, rate=1):

        self.window_size = window_size
        self.order = order
        self.deriv = deriv
        self.rate = rate

        try:
            self.window_size = np.abs(np.int(self.window_size))
            self.order = np.abs(np.int(self.order))
        except ValueError, msg:
            raise ValueError("window_size and order have to be of type int")
        if self.window_size % 2 != 1 or self.window_size < 1:
            raise TypeError("window_size size must be a positive odd number")
        if self.window_size < self.order + 2:
            raise TypeError("window_size is too small for the polynomials order")

    def filter(self, y):

        self.order_range = range(self.order+1)
        half_window = (self.window_size -1) // 2
        # precompute coefficients
        b = np.mat([[k**i for i in self.order_range] for k in range(-half_window, half_window+1)])
        m = np.linalg.pinv(b).A[self.deriv] * self.rate**self.deriv * factorial(self.deriv)
        # pad the signal at the extremes with
        # values taken from the signal itself
        firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
        lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
        y = np.concatenate((firstvals, y, lastvals))
        return np.convolve( m[::-1], y, mode='valid')

if __name__ == "__main__":

    # create some sample twoD data
    x = np.linspace(-3,3,100)
    Z = np.exp( -(x))

    # add noise
    Zn = Z + np.random.normal( 0, 0.2, Z.shape )

    sg = savitzky_golay(window_size=29, order=4)

    l = sg.filter(Zn)

    pylab.plot(l)

    pylab.plot(Zn)

    pylab.show()
