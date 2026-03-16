"""
강화학습 학습 곡선 시각화 스크립트
Usage:
    python python/utils/plot_training.py --run_id autodriver_manual_001
    python python/utils/plot_training.py --run_id autodriver_manual_001 --save
"""

import argparse
import os
import glob
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as ticker
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def load_scalars(tfevents_path: str, tag: str):
    ea = EventAccumulator(tfevents_path)
    ea.Reload()
    events = ea.Scalars(tag)
    steps = [e.step for e in events]
    values = [e.value for e in events]
    return steps, values


def find_tfevents(run_id: str, results_dir: str = "results") -> str:
    pattern = os.path.join(results_dir, run_id, "**", "events.out.tfevents.*")
    files = glob.glob(pattern, recursive=True)
    if not files:
        raise FileNotFoundError(f"tfevents 파일을 찾을 수 없습니다: {pattern}")
    return files[0]


def smooth(values, weight=0.85):
    """지수 이동 평균 스무딩"""
    smoothed = []
    last = values[0]
    for v in values:
        last = last * weight + v * (1 - weight)
        smoothed.append(last)
    return smoothed


def plot(run_id: str, results_dir: str = "results", save: bool = False):
    tfevents_path = find_tfevents(run_id, results_dir)
    print(f"tfevents: {tfevents_path}")

    tags = {
        "reward":   "Environment/Cumulative Reward",
        "ep_len":   "Environment/Episode Length",
        "entropy":  "Policy/Entropy",
        "pol_loss": "Losses/Policy Loss",
        "val_loss": "Losses/Value Loss",
        "lr":       "Policy/Learning Rate",
    }

    data = {}
    for key, tag in tags.items():
        steps, values = load_scalars(tfevents_path, tag)
        data[key] = (steps, values)

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f"Training: {run_id}", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.4, wspace=0.35)

    plots = [
        (gs[0, 0], "reward",   "Cumulative Reward",  "steelblue",  True),
        (gs[0, 1], "ep_len",   "Episode Length",     "darkorange", True),
        (gs[0, 2], "entropy",  "Policy Entropy",     "green",      True),
        (gs[1, 0], "pol_loss", "Policy Loss",        "crimson",    False),
        (gs[1, 1], "val_loss", "Value Loss",         "purple",     False),
        (gs[1, 2], "lr",       "Learning Rate",      "gray",       False),
    ]

    for spec, key, title, color, do_smooth in plots:
        ax = fig.add_subplot(spec)
        steps, values = data[key]
        ax.plot(steps, values, alpha=0.3, color=color, linewidth=0.8)
        if do_smooth and len(values) > 10:
            ax.plot(steps, smooth(values), color=color, linewidth=1.8, label="smoothed")
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("Steps")
        ax.grid(True, alpha=0.3)
        ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x/1000:.0f}k"))
        ax.xaxis.set_major_locator(ticker.MaxNLocator(nbins=5, integer=True))
        ax.tick_params(axis="x", labelsize=8)

        # 최종값 표시
        if values:
            ax.axhline(values[-1], color=color, linestyle="--", linewidth=0.8, alpha=0.6)
            ax.text(steps[-1], values[-1], f" {values[-1]:.3f}", fontsize=8,
                    color=color, va="center")

    # 보상 요약 텍스트
    r_steps, r_vals = data["reward"]
    max_r = max(r_vals)
    final_r = r_vals[-1]
    ax0 = fig.axes[0]
    ax0.set_title(
        f"Cumulative Reward  [max: {max_r:.1f}  final: {final_r:.1f}]",
        fontsize=10
    )

    if save:
        out_path = os.path.join(results_dir, run_id, "training_curve.png")
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"저장됨: {out_path}")
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_id", default="autodriver_manual_001")
    parser.add_argument("--results_dir", default="results")
    parser.add_argument("--save", action="store_true", help="화면 대신 PNG로 저장")
    args = parser.parse_args()

    plot(args.run_id, args.results_dir, args.save)
