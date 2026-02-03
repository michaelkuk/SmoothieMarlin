# Marlin Smoothieboard Configuration

This repository contains Marlin firmware configuration files for Smoothieboard (LPC1769). GitHub Actions automatically builds the firmware and publishes it to GitHub Releases.

## Repository Contents

- `Configuration.h` - Main Marlin configuration
- `Configuration_adv.h` - Advanced Marlin configuration
- `.github/workflows/build.yml` - Automated build workflow

## How It Works

The GitHub Actions workflow:
1. Clones the Marlin `bugfix-2.1.x` branch
2. Copies your configuration files into the Marlin source
3. Builds firmware using PlatformIO for the `LPC1769` environment
4. Uploads `firmware.bin` to GitHub Releases

## Customizing Configuration

1. Edit `Configuration.h` and/or `Configuration_adv.h`
2. Commit and push your changes
3. Trigger a build (see below)

For configuration options, refer to the [Marlin Configuration Documentation](https://marlinfw.org/docs/configuration/configuration.html).

## Triggering Builds

### Option 1: Tag Release (Recommended)

Create and push a version tag:

```bash
git tag v1.0.0
git push --tags
```

This creates a release named after the tag (e.g., "v1.0.0").

### Option 2: Manual Dispatch

1. Go to **Actions** > **Build Marlin Firmware**
2. Click **Run workflow**
3. Optionally enter a version name
4. Click **Run workflow**

## Finding Releases

Built firmware is available at: `https://github.com/<owner>/<repo>/releases`

Download `firmware.bin` and copy it to your Smoothieboard's SD card.

## Build Details

- **Board**: Smoothieboard (BOARD_SMOOTHIEBOARD)
- **Chip**: LPC1769 ARM Cortex-M3
- **Marlin Branch**: bugfix-2.1.x
- **Output**: firmware.bin
