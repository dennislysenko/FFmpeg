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
  --disable-optimizations --disable-static --enable-shared --disable-stripping --disable-mmx --disable-ssse3 --enable-debug=3 --extra-cflags="-O0 -fno-inline"
PATH="$HOME/bin:$PATH" make
make install
make distclean
hash -r

