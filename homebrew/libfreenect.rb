require 'formula'

class Libfreenect <Formula
  # need to define a release so we can do have a known state and md5
  #url 'https://github.com/OpenKinect/libfreenect/tarball/master'
  url 'https://walchko@github.com/walchko/libfreenect.git'
  version 'master'
  homepage 'https://github.com/walchko/libfreenect'
  md5 ''
  
  # libusb 1.09 has patches to work with libfreenect
  depends_on 'libusb'
  depends_on 'cmake'

  def options
  [
    ['--audio',"Build audio interface"],
  ]
  end
  
  def install
    args = std_cmake_parameters.split
    
    #FIXME[13Nov11]: Cant find glut
    if ARGV.include? '--audio'
      args << "-DBUILD_AUDIO:BOOL=ON"
    end
    
    args << ".."
    
    mkdir "build"
    cd "build"
    #system "cmake .. #{std_cmake_parameters}"
    system "cmake", *args
    system "make install"
  end
end
