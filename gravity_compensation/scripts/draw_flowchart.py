#!/usr/bin/env python3
"""
Draw a simple control-loop flowchart (SimpleGravityCompensation) using matplotlib.
Saves `flowchart.png` in the current directory and shows the figure.
"""
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


def draw_box(ax, text, center, width=3.2, height=0.8, facecolor="#F0F8FF"):
    x = center[0] - width / 2.0
    y = center[1] - height / 2.0
    box = FancyBboxPatch((x, y), width, height,
                         boxstyle="round,pad=0.3",
                         linewidth=1.1, facecolor=facecolor, edgecolor="#333")
    ax.add_patch(box)
    ax.text(center[0], center[1], text, ha="center", va="center", fontsize=10)


def draw_arrow(ax, p_from, p_to):
    # shift endpoints slightly to avoid overlapping box borders
    arrow = FancyArrowPatch(p_from, p_to, arrowstyle="-|>",
                            mutation_scale=14, linewidth=1.0, color="#222")
    ax.add_patch(arrow)


def main():
    fig, ax = plt.subplots(figsize=(6, 10))
    ax.set_xlim(-3, 3)
    ax.set_ylim(-10, 2)
    ax.axis("off")

    # vertical positions for boxes
    ys = [1.0, -0.8, -2.2, -3.6, -4.8, -6.0, -7.4, -8.6]
    labels = [
        "Start",
        "Read Encoder\n(turn, vel)",
        "Compute q, dq\n(convert to rad)",
        "Compute Gravity\nTorque (tau_g)",
        "Compute Impedance\nTorque (tau_imp)",
        "Sum & Clamp\n(tau_cmd)",
        "Convert to Motor\nTorque (tau_motor)",
        "Send Torque\nCommand (ODrive)"
    ]

    centers = [(0.0, y) for y in ys]

    # draw boxes
    for c, label in zip(centers, labels):
        draw_box(ax, label, c)

    # draw arrows between boxes
    for i in range(len(centers) - 1):
        top = centers[i]
        bot = centers[i + 1]
        # arrow start slightly below top, end slightly above bottom
        draw_arrow(ax, (top[0], top[1] - 0.4), (bot[0], bot[1] + 0.4))

    ax.set_title("Simple Gravity Compensation - Control Loop", fontsize=12)
    plt.tight_layout()

    out = "flowchart.png"
    plt.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Saved flowchart to {out}")
    plt.show()


if __name__ == "__main__":
    main()
