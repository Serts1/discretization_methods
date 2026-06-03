import numpy as np
import scipy.signal as signal
import scipy.linalg as linalg
import sympy as sp
import matplotlib.pyplot as plt
import math


# ---------------------------------------------------------
# Ψηφιακός Ελεγκτής PID (Digital PID Controller Class)
# ---------------------------------------------------------
class DigitalPID:
    def __init__(self, fp, fi, fd, T):
        self.fp = fp  # Proportional Gain
        self.fi = fi  # Integral Gain
        self.fd = fd  # Derivative Gain
        self.T = T
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.T
        derivative = (error - self.prev_error) / self.T
        output = self.fp * error + self.fi * self.integral + self.fd * derivative
        self.prev_error = error
        return output


# ---------------------------------------------------------
# Αλγόριθμοι Διακριτοποίησης (Numerical Methods)
# ---------------------------------------------------------
def discretize_forward(g, z, n, T):
    af = np.zeros(n + 1)
    bf = np.zeros(n + 1)
    for k in range(1, n + 2):
        i = k - 1
        for j in range(0, i + 1):
            term = ((-1) ** (i - j) * T ** j * math.factorial(n - j)) / \
                   (math.factorial(i - j) * math.factorial(n - i))
            af[k - 1] += term * g[j]
            bf[k - 1] += term * z[j]
    return bf, af


def discretize_backward(g, z, n, T):
    ab = np.zeros(n + 1)
    bb = np.zeros(n + 1)
    for i in range(1, n + 2):
        k = i - 1
        for j in range(0, n - k + 1):
            term = ((-1) ** k * T ** j * math.factorial(n - j)) / \
                   (math.factorial(k) * math.factorial(n - j - k))
            ab[i - 1] += term * g[j]
            bb[i - 1] += term * z[j]
    return bb, ab


def discretize_tustin(g, z, n, T):
    q = sp.Symbol('q')
    at = np.zeros(n + 1)
    bt = np.zeros(n + 1)
    for i in range(n + 1):
        P = (q + 1) ** i * (q - 1) ** (n - i)
        coeffs = np.array([float(c) for c in sp.Poly(P, q).all_coeffs()])
        factor = (2 / T) ** (n - i)
        at += factor * g[i] * coeffs
        bt += factor * z[i] * coeffs
    return bt, at


# ---------------------------------------------------------
# Main Simulation & Visualization
# ---------------------------------------------------------
def main():
    n, T = 2, 0.1
    # Ορισμός συστήματος βάσει των ασκήσεων
    g = np.array([1.0, 0.2, 0.3])
    z = np.array([0.0, 0.0, 1.0])

    # Υπολογισμός συντελεστών για τις 3 μεθόδους
    bf, af = discretize_forward(g, z, n, T)
    bb, ab = discretize_backward(g, z, n, T)
    bt, at = discretize_tustin(g, z, n, T)

    print("--- Υπολογισμός Συντελεστών Διακριτοποίησης ---")
    print(f"Tustin - Αριθμητής: {bt}, Παρονομαστής: {at}")
    print("-" * 50)

    t_cont = np.arange(0, 10, 0.001)
    t_disc = np.arange(0, 10 + T, T)

    # 1. Προσομοίωση Ανοικτού Βρόχου
    # Χρησιμοποιούμε [1.0] αντί για z για να αποφύγουμε το BadCoefficients warning
    sys_c = signal.TransferFunction([1.0], g)
    _, y_cont = signal.step(sys_c, T=t_cont)

    _, y_f = signal.dstep(signal.TransferFunction(bf, af, dt=T), t=t_disc)
    _, y_b = signal.dstep(signal.TransferFunction(bb, ab, dt=T), t=t_disc)
    _, y_t = signal.dstep(signal.TransferFunction(bt, at, dt=T), t=t_disc)

    # 2. Προσομοίωση Κλειστού Βρόχου με Ψηφιακό Ελεγκτή
    sys_ss = signal.TransferFunction(bt, at, dt=T).to_ss()
    x_cl, y_cl = np.zeros((sys_ss.A.shape[0], 1)), np.zeros(len(t_disc))
    pid = DigitalPID(fp=1.5, fi=0.8, fd=0.1, T=T)

    for k in range(len(t_disc)):
        y_cl[k] = (sys_ss.C @ x_cl)[0, 0]
        u = pid.compute(1.0 - y_cl[k])  # Target Step = 1.0
        if k < len(t_disc) - 1:
            x_cl = sys_ss.A @ x_cl + sys_ss.B * u

    # --- Σχεδίαση ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    ax1.plot(t_cont, y_cont, 'k-', linewidth=1.5, label='Continuous System')
    ax1.plot(t_disc, np.squeeze(y_f), 'ro', markersize=4, label='Forward Difference')
    ax1.plot(t_disc, np.squeeze(y_b), 'g^', markersize=4, label='Backward Difference')
    ax1.plot(t_disc, np.squeeze(y_t), 'b*', markersize=5, label='Tustin (Bilinear)')
    ax1.set_title('Open-Loop Step Response Comparison');
    ax1.legend();
    ax1.grid(True)

    ax2.plot(t_disc, y_cl, 'g-o', label='Closed-Loop (Digital PID Control)')
    ax2.axhline(1, color='r', ls='--', label='Reference Signal')
    ax2.set_title('Closed-Loop Response with PID (fp=1.5, fi=0.8)');
    ax2.legend();
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('step_response_comparison.png', dpi=300)
    plt.show()


if __name__ == "__main__":
    main()