#!/usr/bin/env python3
"""
Draw a detailed control-loop flowchart for SimpleGravityCompensation using matplotlib.
Saves `flowchart_detailed.png` and shows the figure.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Polygon


def draw_box(ax, text, center, width=3.6, height=0.9, facecolor="#EFEFEF"):
    x = center[0] - width / 2.0
    y = center[1] - height / 2.0
    box = FancyBboxPatch((x, y), width, height,
                         boxstyle="round,pad=0.3",
                         linewidth=1.0, facecolor=facecolor, edgecolor="#333")
    ax.add_patch(box)
    ax.text(center[0], center[1], text, ha="center", va="center", fontsize=9)


def draw_diamond(ax, text, center, width=2.2, height=1.0, facecolor="#FFF4CC"):
    cx, cy = center
    pts = [
        (cx, cy + height / 2.0),
        (cx + width / 2.0, cy),
        (cx, cy - height / 2.0),
        (cx - width / 2.0, cy),
    ]
    poly = Polygon(pts, closed=True, facecolor=facecolor, edgecolor="#333")
    ax.add_patch(poly)
    ax.text(cx, cy, text, ha="center", va="center", fontsize=9)


def draw_arrow(ax, p_from, p_to, text=None, text_offset=(0,0)):
    arrow = FancyArrowPatch(p_from, p_to, arrowstyle="-|>",
                            mutation_scale=12, linewidth=1.0, color="#222")
    ax.add_patch(arrow)
    if text:
        mx = 0.5*(p_from[0]+p_to[0]) + text_offset[0]
        my = 0.5*(p_from[1]+p_to[1]) + text_offset[1]
        ax.text(mx, my, text, fontsize=8, ha='center', va='center', backgroundcolor='white')


def main():
    fig, ax = plt.subplots(figsize=(7, 10))
    ax.set_xlim(-4, 4)
    ax.set_ylim(-11, 2)
    ax.axis('off')

    # layout y positions
    y = {}
    y['start'] = 1.0
    y['init'] = -0.6
    y['read'] = -2.0
    y['convert'] = -3.2
    y['gravity'] = -4.6
    y['impedance'] = -6.0
    y['sum'] = -7.4
    y['convert_motor'] = -8.8
    y['send'] = -10.2

    # draw boxes
    draw_box(ax, 'Start / Node Init\n- load params\n- init ODrive\n- set modes', (0.0, y['start']))
    draw_box(ax, 'Initialization\nDeclare joint params\nLoad JointConfig', (0.0, y['init']))
    draw_box(ax, 'Read Encoder\nget_encoder_data(turn, vel_turn_s)', (0.0, y['read']))
    draw_box(ax, 'Compute q, dq\nq = turn*REV_TO_RAD/gear\ndq LPF', (0.0, y['convert']))
    draw_box(ax, 'Compute Gravity\n tau_g = mass*g*L*sin(q + initial_angle)', ( -1.6, y['gravity']))
    draw_box(ax, 'Compute Impedance\n tau_imp = -K*(q - q_ref) - B*dq', (1.6, y['impedance']))

    # sum/clamp
    draw_box(ax, 'Sum & Clamp\n tau_cmd = clamp(tau_g + tau_imp, -max, +max)\n handle NaN', (0.0, y['sum']))

    # convert & send
    draw_box(ax, 'Convert to Motor Torque\n motor_tau = tau_cmd / gear', (0.0, y['convert_motor']))
    draw_box(ax, 'Send Torque Command\n odrive.SendTorqueCommand(id, motor_tau)\nLog periodically', (0.0, y['send']))

    # decision: auto_update_q_ref?
    draw_diamond(ax, 'auto_update_q_ref?', (2.8, y['convert']))
    draw_diamond(ax, 'abs(dq) < dq_thresh?', (2.8, y['gravity']))

    # arrows vertical main flow
    draw_arrow(ax, (0.0, y['start']-0.45), (0.0, y['init']+0.45))
    draw_arrow(ax, (0.0, y['init']-0.45), (0.0, y['read']+0.45))
    draw_arrow(ax, (0.0, y['read']-0.45), (0.0, y['convert']+0.45))

    # branch to gravity & impedance (side-by-side)
    draw_arrow(ax, (0.0, y['convert']-0.45), (-1.6, y['gravity']+0.45))
    draw_arrow(ax, (0.0, y['convert']-0.45), (1.6, y['impedance']+0.45))

    # from gravity & impedance to sum
    draw_arrow(ax, (-1.6, y['gravity']-0.45), (0.0, y['sum']+0.45))
    draw_arrow(ax, (1.6, y['impedance']-0.45), (0.0, y['sum']+0.45))

    # sum -> convert_motor -> send
    draw_arrow(ax, (0.0, y['sum']-0.45), (0.0, y['convert_motor']+0.45))
    draw_arrow(ax, (0.0, y['convert_motor']-0.45), (0.0, y['send']+0.45))

    # auto_update branch connect near convert/dq lpf
    draw_arrow(ax, (0.0, y['convert']-0.15), (2.8, y['convert']+0.15), text='check param', text_offset=(0.6,0.0))

    # diamond flow: auto_update_q_ref? -> if yes check dq small
    draw_arrow(ax, (2.8 - 1.0, y['convert'] - 0.4), (2.8, y['convert'] + 0.42))
    draw_arrow(ax, (2.8, y['convert'] - 0.58), (2.8, y['gravity'] + 0.58), text='true')
    draw_arrow(ax, (2.8, y['convert'] - 0.58), (4.6, y['convert'] - 0.58), text='false')

    # dq check diamond -> if true update q_ref -> join back to main (no-op path shown)
    draw_arrow(ax, (2.8, y['gravity'] - 0.58), (2.8, y['gravity'] + 0.58))
    draw_arrow(ax, (2.8, y['gravity'] - 0.58), (1.2, y['convert'] - 0.4), text='true')
    draw_arrow(ax, (2.8, y['gravity'] - 0.58), (4.6, y['gravity'] - 0.58), text='false')

    # small annotation: per-joint loop
    ax.text(-3.2, -9.6, 'Loop over joints: read -> compute -> send', fontsize=8, color='#333')

    ax.set_title('Detailed Flowchart: SimpleGravityCompensation Node', fontsize=12)
    plt.tight_layout()

    out = 'flowchart_detailed.png'
    plt.savefig(out, dpi=200, bbox_inches='tight')
    print(f'Saved {out}')
    plt.show()

if __name__ == '__main__':
    main()
