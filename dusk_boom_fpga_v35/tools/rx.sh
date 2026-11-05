# rx.sh OUT SIZE SHA256 - receive one file over the console with busybox only (same protocol as rx.py)
out=$1; size=$2; want=$3; n=0; : > $out.part
echo READY
while IFS=' ' read -r tag idx md5 b64; do
  [ "$tag" = END ] && break
  if [ "$tag" = D ] && [ "$idx" = "$n" ]; then
    printf '%s' "$b64" | base64 -d > /tmp/rx.chunk 2>/dev/null
    if [ "$(md5sum < /tmp/rx.chunk | cut -d' ' -f1)" = "$md5" ]; then cat /tmp/rx.chunk >> $out.part; echo "A $n"; n=$((n+1)); continue; fi
  fi
  echo "N $n"
done
got=$(wc -c < $out.part); sha=$(sha256sum < $out.part | cut -d' ' -f1)
if [ "$got" = "$size" ] && [ "$sha" = "$want" ]; then mv $out.part $out; echo "RX_OK $out $got"; else echo "RX_FAIL size=$got sha=$sha"; fi
