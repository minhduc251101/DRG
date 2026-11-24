#!/bin/bash
# Score every released trajectory against the ground truth with evo_ape -va (SE(3) Umeyama alignment)
# and save evo's results (.zip), plot (.pdf) and printed output (.txt) next to it.
#
#   bash scripts/evo_ape_all.sh                 # needs evo (pip install evo); run anywhere
#   bash scripts/evo_ape_all.sh trajectories    # or slam_backend: only that folder
#
# For each folder it runs, e.g. in results/trajectories/loop1:
#   evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --save_results evo_ape.zip --save_plot evo_ape.pdf --plot_mode xy
cd "$(dirname "$0")/../results" || exit 1

run() {  # <folder> <ground truth> <estimate> <output name>
  local cmd="evo_ape tum $2 $3 -va --save_results $4.zip --save_plot $4.pdf --plot_mode xy"
  ( cd "$1" && rm -f "$4.zip" "$4.pdf" \
    && { echo "\$ $cmd"; $cmd --no_warnings; } > "$4.txt" 2>&1 ) || { echo "FAILED: $1/$4 (see $1/$4.txt)"; return 1; }
  printf "%-32s %-28s APE rmse %s m\n" "$1" "$4" "$(awk '$1 == "rmse" {print $2}' "$1/$4.txt")"
}

PART=${1:-all}
[ "$PART" = all ] || [ "$PART" = trajectories ] || [ "$PART" = slam_backend ] || { echo "usage: $0 [trajectories|slam_backend]"; exit 1; }
[ "$PART" = slam_backend ] || for seq in cp garden nyl loop1 loop2 loop3 garden_every_scan; do
  run "trajectories/$seq" stamp_gt_traj.txt stamp_est_traj.txt evo_ape
  if [ -f "trajectories/$seq/stamp_est_traj_segmented.txt" ]; then
    run "trajectories/$seq" stamp_gt_traj.txt stamp_est_traj_segmented.txt evo_ape_segmented
  fi
done
[ "$PART" = trajectories ] || for seq in cp garden nyl loop1 loop2 loop3; do
  for est in front_end no_loop_closure loop_closure; do
    gt=stamp_gt_traj.txt   # the keyframes of the loop-closure run; the other run's, if they differ
    [ -f "slam_backend/$seq/stamp_gt_traj_$est.txt" ] && gt="stamp_gt_traj_$est.txt"
    run "slam_backend/$seq" "$gt" "stamp_est_traj_$est.txt" "evo_ape_$est"
  done
done
