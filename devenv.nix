{ pkgs, lib, config, ... }:

{
  languages.zig.enable = true;

  # This is needed for libc headers
  languages.c.enable = true;

  packages = with pkgs; [
    wayland
    egl-wayland
    libxkbcommon
    xorg.libX11
    xorg.libXext
    xorg.libXcursor
    xorg.libXi
    xorg.libXfixes
    xorg.libXrandr
    libpulseaudio
    vulkan-headers
    vulkan-loader
  ];

  enterShell = ''
    zig version
  '';

  # See full reference at https://devenv.sh/reference/options/
}
