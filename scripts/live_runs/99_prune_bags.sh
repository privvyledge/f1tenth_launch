#!/usr/bin/env bash
# 99_prune_bags.sh — interactively delete bags after you have inspected them.
#
#   ./99_prune_bags.sh              # list bags under today's BAG_ROOT
#   ./99_prune_bags.sh --all-dates  # every bag under $SSD_ROOT/bags
#
# Never deletes without showing you what it is about to remove and getting an
# explicit confirmation per bag. There is no --force and no glob-delete: a bag
# you cannot re-record is worth more than the disk it occupies.

set -uo pipefail
cd "$(dirname "$0")"
# shellcheck source=00_env.sh
source ./00_env.sh

SEARCH_ROOT="$BAG_ROOT"
[[ "${1:-}" == "--all-dates" ]] && SEARCH_ROOT="${SSD_ROOT}/bags"

[[ -d "$SEARCH_ROOT" ]] || die "no bag directory at $SEARCH_ROOT"

print_env
banner "bags under $SEARCH_ROOT"

# rosbag2 writes a directory containing metadata.yaml.
mapfile -t BAGS < <(find "$SEARCH_ROOT" -name metadata.yaml -printf '%h\n' 2>/dev/null | sort)

if (( ${#BAGS[@]} == 0 )); then
  info "no bags found — nothing to prune"
  exit 0
fi

for i in "${!BAGS[@]}"; do
  printf '  [%2d] %-56s %s\n' \
    "$i" "${BAGS[$i]#"$SEARCH_ROOT"/}" "$(du -sh "${BAGS[$i]}" 2>/dev/null | cut -f1)"
done

banner "disk"
df -h "$SSD_ROOT" 2>/dev/null | sed 's/^/  /'

banner "prune"
warn "Inspect a bag before deleting it:  ./90_inspect_bag.sh <path>"
cat <<'EOF'
  Enter indices to DELETE (space separated), or press Enter to keep everything.
EOF
read -r -p "  delete> " -a PICKS

(( ${#PICKS[@]} == 0 )) && { info "nothing deleted"; exit 0; }

TO_DELETE=()
for p in "${PICKS[@]}"; do
  [[ "$p" =~ ^[0-9]+$ ]] || die "not an index: $p"
  (( p < ${#BAGS[@]} )) || die "index out of range: $p"
  TO_DELETE+=("${BAGS[$p]}")
done

banner "about to delete"
for b in "${TO_DELETE[@]}"; do
  printf '  %s  (%s)\n' "$b" "$(du -sh "$b" 2>/dev/null | cut -f1)"
done

read -r -p $'\n  Type \'delete\' to confirm: ' reply
[[ "$reply" == "delete" ]] || { info "aborted — nothing deleted"; exit 0; }

for b in "${TO_DELETE[@]}"; do
  # Guard against a path outside the bag tree, however it got here.
  case "$b" in
    "${SSD_ROOT}"/bags/*) rm -rf -- "$b" && info "deleted $b" ;;
    *) err "refusing to delete outside ${SSD_ROOT}/bags: $b" ;;
  esac
done

banner "disk after"
df -h "$SSD_ROOT" 2>/dev/null | sed 's/^/  /'
