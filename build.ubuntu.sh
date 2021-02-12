sudo apt install make gcc yasm libass-dev libtheora-dev libvorbis-dev libssl-dev 

echo "If you haven't already, follow this tutorial to build libx264: https://serverok.in/error-x264_bit_depth-undeclared"

PATH="$HOME/bin:$PATH" PKG_CONFIG_PATH="$HOME/ffmpeg_build/lib/pkgconfig" ./configure \
  --prefix="$HOME/ffmpeg_build" \
  --pkg-config-flags="--static" \
  --extra-cflags="-I$HOME/ffmpeg_build/include" \
  --extra-ldflags="-L$HOME/ffmpeg_build/lib" \
  --bindir="$HOME/bin" \
  --enable-gpl \
  --enable-libass \
  --enable-libfreetype \
  --enable-libtheora \
  --enable-libvorbis \
  --enable-libx264 \
  --enable-nonfree \
  --enable-openssl

PATH="$HOME/bin:$PATH" make
make install
make distclean
hash -r

