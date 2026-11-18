#!/usr/bin/env python3
"""
Draw a flowchart for node_gravityCompensation_elbowJoint.cpp
Saves `flowchart_elbow.png` in current directory.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


def draw_box(ax, text, center, width=3.6, height=0.9, facecolor="#E8F8F5"):
    x = center[0] - width/2.0
    y = center[1] - height/2.0
    box = FancyBboxPatch((x,y), width, height, boxstyle="round,pad=0.3", linewidth=1.0, facecolor=facecolor, edgecolor="#333")
    ax.add_patch(box)
    ax.text(center[0], center[1], text, ha='center', va='center', fontsize=9)


def draw_arrow(ax, p_from, p_to, text=None, text_offset=(0,0)):
    a = FancyArrowPatch(p_from, p_to, arrowstyle='-|>', mutation_scale=12, linewidth=1.0, color='#222')
    ax.add_patch(a)
    if text:
        mx = 0.5*(p_from[0]+p_to[0]) + text_offset[0]
        my = 0.5*(p_from[1]+p_to[1]) + text_offset[1]
        ax.text(mx, my, text, fontsize=8, ha='center', va='center', backgroundcolor='white')


def main():
    fig, ax = plt.subplots(figsize=(7, 10))
    ax.set_xlim(-4, 4)
    ax.set_ylim(-11, 2)
    ax.axis('off')

    ys = {
        'start': 1.0,
        'params': -0.6,
        'init_odrive': -1.8,
        'timer': -3.0,
        'read': -4.2,
        'compute_q': -5.4,
        'gravity': -6.6,
        'delta_tau': -7.6,
        'admittance': -8.6,
        'feedforward': -9.6,
        'sum_send': -10.6
    }

    draw_box(ax, 'Start / Node Init\n- declare parameters\n- create ODrive interface', (0.0, ys['start']))
    draw_box(ax, 'initialize_odrive()\n- InitVelocityMode(MOTOR_ID)\n- SendVelocityCommand(MOTOR_ID, 0.0)', (0.0, ys['init_odrive']))
    draw_box(ax, 'Start Timer (1ms)\npid_admittance_callback_loop()', (0.0, ys['timer']))

    draw_box(ax, 'Read sensors\nodrive.ReadEncoder(id)\nodrive.ReadTorque(id)\nget_encoder_data_nolog()', (0.0, ys['read']))

    draw_box(ax, 'Compute q, dq\nq = turn * REV_TO_RAD / GEAR\ndq = vel_rev_s * REV_TO_RAD / GEAR', (0.0, ys['compute_q']))

    draw_box(ax, 'Gravity Compensation\n tau_g = robot_mass * g * com_dist * sin(q)\nreturn tau_g', (-1.6, ys['gravity']))

    draw_box(ax, 'Delta Tau\n delta_tau_theoretical = tau_meas - tau_g\ndelta_tau = -delta_tau_theoretical', (1.6, ys['delta_tau']))

    draw_box(ax, 'PidAdmittanceBlock(delta_tau)\n- qdd = inv(M)*(tau_ext - B*qd - K*(q_cmd - q_ref))\n- qd += qdd*dt\n- clamp qd\n- q_cmd += qd*dt\n- return qd', (0.0, ys['admittance']))

    draw_box(ax, 'Velocity Feedforward\n velocity_ff = velocity_feedforward_gain * dq', (2.8, ys['feedforward']))

    draw_box(ax, 'Sum Velocities & Send\n total_vel = qd_cmd + velocity_ff\nrev/s = total_vel * RAD_TO_REV\nodrive.SendVelocityCommand(id, rev/s)', (0.0, ys['sum_send']))

    # arrows
    draw_arrow(ax, (0.0, ys['start']-0.45), (0.0, ys['init_odrive']+0.45))
    draw_arrow(ax, (0.0, ys['init_odrive']-0.45), (0.0, ys['timer']+0.45))
    draw_arrow(ax, (0.0, ys['timer']-0.45), (0.0, ys['read']+0.45))
    draw_arrow(ax, (0.0, ys['read']-0.45), (0.0, ys['compute_q']+0.45))

    # from compute to gravity and delta path
    draw_arrow(ax, (0.0, ys['compute_q']-0.45), (-1.6, ys['gravity']+0.45))
    draw_arrow(ax, (0.0, ys['compute_q']-0.45), (1.6, ys['delta_tau']+0.45))

    # gravity -> delta_tau (join)
    draw_arrow(ax, (-1.6, ys['gravity']-0.45), (0.0, ys['admittance']+0.45), text='tau_g')
    draw_arrow(ax, (1.6, ys['delta_tau']-0.45), (0.0, ys['admittance']+0.45), text='delta_tau')

    # admittance -> sum
    draw_arrow(ax, (0.0, ys['admittance']-0.45), (0.0, ys['sum_send']+0.45), text='qd_cmd')

    # feedforward -> sum
    draw_arrow(ax, (2.8, ys['feedforward']-0.45), (0.8, ys['sum_send']+0.45), text='velocity_ff')

    # small annotation
    ax.text(-3.2, -10.3, 'Constants: REV_TO_RAD = M_PI/4, RAD_TO_REV = 4/M_PI, dt=0.001', fontsize=8, color='#333')

    ax.set_title('Flowchart: Gravity Compensation (Elbow) Node', fontsize=12)
    plt.tight_layout()

    out = 'flowchart_elbow.png'
    plt.savefig(out, dpi=200, bbox_inches='tight')
    print('Saved', out)
    plt.show()

if __name__ == '__main__':
    main()
