require 'formula'

# Seems to compile fine with --use-gcc option

class Pcl < Formula
  #url 'http://dev.pointclouds.org/attachments/download/610/PCL-1.4.0-Source.tar.bz2'
  #md5 '63fd633a6306ae9b334131b250a2f893'
  #version '1.4'
  
  #url 'http://www.pointclouds.org/assets/files/1.5.1/PCL-1.5.1-Source.tar.bz2'
  #md5 'd96479ab65245c64d91a6fe1d803275a'
  #version '1.5.1'
  
  #url 'http://www.pointclouds.org/assets/files/1.6.0/PCL-1.6.0-Source.tar.bz2'
  #md5 'f83ca5d0ff290412b0807864b95eba26'
  #version '1.6.0'
  
  # brew install pcl --devel
  #devel do
    url 'http://svn.pointclouds.org/pcl/trunk', :using => :svn
    version '1.7.0'
  #end
  
  homepage 'http://www.pointclouds.org'
  
  depends_on 'cmake'
  # ROS needs boost 1.49 to compile, it doesn't work with 1.50
  # git checkout e40bc41 /usr/local/Library/Formula/boost.rb
  #depends_on 'boost'
  depends_on 'eigen'
  depends_on 'flann'
  depends_on 'vtk'
  depends_on 'qhull'
  depends_on 'libusb'

  depends_on 'doxygen' if ARGV.include? '--doc'
  depends_on 'sphinx' if ARGV.include? '--doc'

  def options
  [
    ['--apps', "Build apps"],
    ['--doc', "Build documentation"],
    ['--nofeatures', "Disable features module"],
    ['--nofilters', "Disable filters module"],
    ['--noio', "Disable io module"],
    ['--nokdtree', "Disable kdtree module"],
    ['--nokeypoints', "Disable keypoints module"],
    ['--nooctree', "Disable octree module"],
    ['--noproctor', "Disable proctor module"],
    ['--nopython', "Disable Python bindings"],
    ['--norangeimage', "Disable range image module"],
    ['--noregistration', "Disable registration module"],
    ['--nosac', "Disable sample consensus module"],
    ['--nosearch', "Disable search module"],
    ['--nosegmentation', "Disable segmentation module"],
    ['--nosurface', "Disable surface module"],
    ['--notools', "Disable tools module"],
    ['--notracking', "Disable tracking module"],
    ['--novis', "Disable visualisation module"],
    ['--ros', "Enable ROS module"],
    ['--openni', "Enable OpenNI module"],
  ]
  end

  def install
    args = std_cmake_parameters.split

    if ARGV.include? '--apps'
      args << "-DBUILD_apps:BOOL=ON"
    else
      args << "-DBUILD_apps:BOOL=OFF"
    end
    if ARGV.include? '--doc'
      args << "-DBUILD_documentation:BOOL=ON"
    else
      args << "-DBUILD_documentation:BOOL=OFF"
    end
    if ARGV.include? '--nofeatures'
      args << "-DBUILD_features:BOOL=OFF"
    end
    if ARGV.include? '--nofilters'
      args << "-DBUILD_filters:BOOL=OFF"
    end
    if ARGV.include? '--noio'
      args << "-DBUILD_io:BOOL=OFF"
    end
    if ARGV.include? '--nokdtree'
      args << "-DBUILD_kdtree:BOOL=OFF"
    end
    if ARGV.include? '--nokeypoints'
      args << "-DBUILD_keypoints:BOOL=OFF"
    end
    if ARGV.include? '--nooctree'
      args << "-DBUILD_octree:BOOL=OFF"
    end
    if ARGV.include? '--noproctor'
      args << "-DBUILD_proctor:BOOL=OFF"
    end
    if ARGV.include? '--nopython'
      args << "-DBUILD_python:BOOL=OFF"
    end
    if ARGV.include? '--norangeimage'
      args << "-DBUILD_rangeimage:BOOL=OFF"
    end
    if ARGV.include? '--noregistration'
      args << "-DBUILD_registration:BOOL=OFF"
    end
    if ARGV.include? '--nosac'
      args << "-DBUILD_sac:BOOL=OFF"
    end
    if ARGV.include? '--nosearch'
      args << "-DBUILD_search:BOOL=OFF"
    end
    if ARGV.include? '--nosegmentation'
      args << "-DBUILD_segmentation:BOOL=OFF"
    end
    if ARGV.include? '--nosurface'
      args << "-DBUILD_surface:BOOL=OFF"
    end
    if ARGV.include? '--notools'
      args << "-DBUILD_tools:BOOL=OFF"
    end
    if ARGV.include? '--notracking'
      args << "-DBUILD_tracking:BOOL=OFF"
    end
    if ARGV.include? '--novis'
      args << "-DBUILD_visualization:BOOL=OFF"
    end
    if ARGV.include? '--ros'
      args << "-DUSE_ROS:BOOL=ON"
    else
      args << "-DUSE_ROS:BOOL=OFF"
    end
    if ARGV.include? '--openni'
      # May need to change this if different location
      args << "-DOPENNI_INCLUDE_DIR=/usr/local/include/ni"
    end

    system "mkdir build"
    args << ".."
    Dir.chdir 'build' do
      system "cmake", *args
      system "make install"
    end
  end

  def test
    # This test will fail and we won't accept that! It's enough to just
    # replace "false" with the main program this formula installs, but
    # it'd be nice if you were more thorough. Test the test with
    # `brew test pcl`. Remove this comment before submitting
    # your pull request!
    system "false"
  end
end
