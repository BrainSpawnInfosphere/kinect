require 'formula'

class Cvblob < Formula
  url 'http://cvblob.googlecode.com/files/cvblob-0.10.4-src.tgz'
  version "0.10.4"
  homepage 'http://code.google.com/p/cvblob/'
  md5 '9d5e360c6de6fce36e95f5d64b67b9b1'
  
  depends_on "opencv"

  def install
	 system "echo #{prefix}"
	 system "mkdir build"
	 
	 Dir.chdir 'build' do
	 	system "cmake .. -DCMAKE_INSTALL_PREFIX=#{prefix}"
    	system "make install"
    end
  end

end
