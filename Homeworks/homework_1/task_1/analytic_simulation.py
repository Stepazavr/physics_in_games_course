import sympy as sp

# Упрощенная версия задачи 1:
# принято k1 = k2 = k и введен параметр w, где w^2 = k/m.
# В результате система переписана через одну частотную переменную w,
# что ускоряет символьное решение в sympy.dsolve.


def main():
    sp.init_printing()

    t = sp.Symbol('t', real=True)
    m = sp.Symbol('m', positive=True, real=True)
    w = sp.Symbol('w', positive=True, real=True)
    alpha = sp.pi / 6  # 30 degrees
    sa = sp.sin(alpha)
    ca = sp.cos(alpha)

    # Coordinate system: x to the right, y upward.
    x = sp.Function('x')(t)
    y = sp.Function('y')(t)

    # Corrected system from task 1:
    # m*x'' = -sin(a)^2*k2*x + cos(a)sin(a)*k2*y
    # m*y'' =  cos(a)sin(a)*k2*x - (k1 + cos(a)^2*k2)*y
    # With k1 = k2 = k and k/m = w^2:
    # x'' = -sin(a)^2*w^2*x + cos(a)sin(a)*w^2*y
    # y'' =  cos(a)sin(a)*w^2*x - (1 + cos(a)^2)*w^2*y
    ode_x = sp.Eq(
        sp.diff(x, t, 2),
        -sa**2 * w**2 * x + ca * sa * w**2 * y,
    )
    ode_y = sp.Eq(
        sp.diff(y, t, 2),
        ca * sa * w**2 * x - (1 + ca**2) * w**2 * y,
    )

    print('Система уравнений движения в параметре w:')
    sp.pprint(ode_x)
    sp.pprint(ode_y)

    B = sp.Matrix([
        [-sa**2 * w**2, ca * sa * w**2],
        [ca * sa * w**2, -(1 + ca**2) * w**2],
    ])
    print('\nМатрица системы q" = B*q:')
    sp.pprint(sp.simplify(B))

    eigvals = sp.simplify(B).eigenvals()
    print('\nСобственные значения матрицы B:')
    print(eigvals)

    x0, y0, vx0, vy0 = sp.symbols('x0 y0 vx0 vy0', real=True)
    ics = {
        x.subs(t, 0): x0,
        y.subs(t, 0): y0,
        sp.diff(x, t).subs(t, 0): vx0,
        sp.diff(y, t).subs(t, 0): vy0,
    }

    print('\nРешение системы через sympy.dsolve...')
    sol = sp.dsolve([ode_x, ode_y], ics=ics)
    x_sol = sol[0].rhs
    y_sol = sol[1].rhs

    print('\nАналитическое решение (общее с начальными условиями):')
    print('x(t) =')
    sp.pprint(x_sol)
    print('y(t) =')
    sp.pprint(y_sol)

    # Numeric simulation and plots.
    # Example: m = 1, k = 20 -> w = sqrt(k/m) = sqrt(20).
    params = {
        m: 1.0,
        w: float(sp.sqrt(20)),
        x0: 0.08,
        y0: 0.04,
        vx0: 0.0,
        vy0: 0.0,
    }

    x_num = sp.lambdify(t, x_sol.subs(params), modules='numpy')
    y_num = sp.lambdify(t, y_sol.subs(params), modules='numpy')
    vx_num = sp.lambdify(t, sp.diff(x_sol, t).subs(params), modules='numpy')
    vy_num = sp.lambdify(t, sp.diff(y_sol, t).subs(params), modules='numpy')

    import numpy as np
    import matplotlib.pyplot as plt

    ts = np.linspace(0.0, 12.0, 1400)
    xs = np.asarray(x_num(ts), dtype=float)
    ys = np.asarray(y_num(ts), dtype=float)
    vxs = np.asarray(vx_num(ts), dtype=float)
    vys = np.asarray(vy_num(ts), dtype=float)

    m_val = params[m]
    w_val = params[w]
    sa_val = float(sa)
    ca_val = float(ca)

    T = 0.5 * m_val * (vxs**2 + vys**2)

    # Since k = m*w^2, the potential becomes:
    # U = 0.5*m*w^2*sin^2(a)*x^2 - m*w^2*cos(a)sin(a)*x*y
    #     + 0.5*m*w^2*(1+cos^2(a))*y^2
    U = (
        0.5 * m_val * w_val**2 * (sa_val**2) * xs**2
        - m_val * w_val**2 * ca_val * sa_val * xs * ys
        + 0.5 * m_val * w_val**2 * (1 + ca_val**2) * ys**2
    )
    E = T + U

    # Energy axis settings:
    # - mode='auto': limits are computed from data with padding
    # - mode='fixed': use E_AXIS_MIN_J..E_AXIS_MAX_J
    E_AXIS_MODE = 'auto'
    E_AXIS_MIN_J = 0.0
    E_AXIS_MAX_J = 0.05
    E_AXIS_PADDING_REL = 0.12
    E_AXIS_MIN_SPAN_J = 1e-4

    if E_AXIS_MODE == 'fixed':
        e_min_plot = E_AXIS_MIN_J
        e_max_plot = E_AXIS_MAX_J
    else:
        e_min_plot = float(np.min(E))
        e_max_plot = float(np.max(E))
        span = e_max_plot - e_min_plot
        if span < E_AXIS_MIN_SPAN_J:
            center = 0.5 * (e_min_plot + e_max_plot)
            half = 0.5 * E_AXIS_MIN_SPAN_J
            e_min_plot = center - half
            e_max_plot = center + half
            span = E_AXIS_MIN_SPAN_J
        pad = E_AXIS_PADDING_REL * span
        e_min_plot -= pad
        e_max_plot += pad

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    axes[0].plot(xs, ys, color='tab:blue', lw=1.6)
    axes[0].set_title('Траектория точки')
    axes[0].set_xlabel('x, м')
    axes[0].set_ylabel('y, м')
    axes[0].axis('equal')
    axes[0].grid(True, alpha=0.35)

    axes[1].plot(ts, E, color='tab:red', lw=1.6)
    axes[1].set_title('Полная энергия E(t)')
    axes[1].set_xlabel('t, с')
    axes[1].set_ylabel('E, Дж')
    axes[1].set_ylim(e_min_plot, e_max_plot)
    axes[1].grid(True, alpha=0.35)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
