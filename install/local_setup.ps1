# generated from colcon_powershell/shell/template/prefix.ps1.em

# This script extends the environment with all packages contained in this
# prefix path.

# use the Python executable known at configure time
$_colcon_python_executable="/usr/bin/python3"
# allow overriding it with a custom location
if ($env:COLCON_PYTHON_EXECUTABLE) {
  $_colcon_python_executable="$env:COLCON_CURRENT_PREFIX"
}
# if the Python executable doesn't exist try another fall back
if (!(Test-Path "$_colcon_python_executable" -PathType Leaf)) {
  if (Get-Command "python" -ErrorAction SilentlyContinue) {
    $_colcon_python_executable="python"
  } else {
    echo "error: unable to find fallback Python executable"
    return 1
  }
}

# function to source another script with conditional trace output
# first argument: the path of the script
function _colcon_prefix_powershell_source_script {
  param (
    $_colcon_prefix_powershell_source_script_param
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_prefix_powershell_source_script_param) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_prefix_powershell_source_script_param'"
    }
    . "$_colcon_prefix_powershell_source_script_param"
  } else {
    Write-Error "not found: '$_colcon_prefix_powershell_source_script_param'"
  }
}

# get all packages in topological order
$_colcon_ordered_packages = & "$_colcon_python_executable" "$(Split-Path $PSCommandPath -Parent)/_local_setup_util.py"

# source package specific scripts in topological order
ForEach ($_colcon_package_name in $($_colcon_ordered_packages -split "`r`n"))
{
  # setting COLCON_CURRENT_PREFIX avoids relying on the build time prefix of the sourced script
  $env:COLCON_CURRENT_PREFIX=(Split-Path $PSCommandPath -Parent) + "\$_colcon_package_name"
  _colcon_prefix_powershell_source_script "$env:COLCON_CURRENT_PREFIX\share\$_colcon_package_name\package.ps1"
}
