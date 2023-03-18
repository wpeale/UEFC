import json
import numpy as np
import matplotlib.pyplot as plt
import math

def gen_graph(file, title):
    with open(file) as f:
        data = f.read()

    js = json.loads(data)
    lamb_dup = [js[k][0][3] for k in js]
    tau_dup = np.array([js[k][0][2] for k in js])
    tau = np.unique(tau_dup)
    lamb  = np.unique(lamb_dup)
    L, T = np.meshgrid(lamb, tau)

    mO3 = np.ndarray((15,15))
    for t in range(15):
        ta = tau[t]
        for l in range(15):
            la = lamb[l]
            for k in js:
                if math.isclose(js[k][0][3],la) and math.isclose(js[k][0][2],ta):
                    mO3[l][t] = float(k)

    _, ax = plt.subplots()
    levels = np.linspace(np.min(mO3[mO3 > 0]), np.max(mO3), 21)
    cs = ax.contour(T, L, mO3, levels=levels, linewidths=0.5)
    plt.clabel(cs, inline=1, fontsize=12)
    ax.set_xlabel(r'$\tau$ ($\frac{t}{c}$)')
    ax.set_ylabel(r'$\lambda$ ($\frac{c_t}{c_r}$)')
    ax.set_title(title)


gen_graph('Optimal_ARS_3.txt', r'$m_{\mathrm{pay}}\Omega^3$ $(\frac{\mathrm{g}}{\mathrm{s}^3})$ Contour Plot (Question 4)')
gen_graph('Optimal_ARS_2.txt', r'$m_{\mathrm{pay}}\Omega^3$ $(\frac{\mathrm{g}}{\mathrm{s}^3})$ Contour Plot (Question 5)')
gen_graph('Optimal_ARS_1.txt', r'$m_{\mathrm{pay}}\Omega^3$ $(\frac{\mathrm{g}}{\mathrm{s}^3})$ Contour Plot (Question 6)')
plt.show()