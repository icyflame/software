module Paths_ASLAM (
    version,
    getBinDir, getLibDir, getDataDir, getLibexecDir,
    getDataFileName, getSysconfDir
  ) where

import qualified Control.Exception as Exception
import Data.Version (Version(..))
import System.Environment (getEnv)
import Prelude

catchIO :: IO a -> (Exception.IOException -> IO a) -> IO a
catchIO = Exception.catch


version :: Version
version = Version {versionBranch = [0,1,0,0], versionTags = []}
bindir, libdir, datadir, libexecdir, sysconfdir :: FilePath

bindir     = "/home/binarymage/cuauv/software/ASLAM/.cabal-sandbox/bin"
libdir     = "/home/binarymage/cuauv/software/ASLAM/.cabal-sandbox/lib/x86_64-linux-ghc-7.8.3/ASLAM-0.1.0.0"
datadir    = "/home/binarymage/cuauv/software/ASLAM/.cabal-sandbox/share/x86_64-linux-ghc-7.8.3/ASLAM-0.1.0.0"
libexecdir = "/home/binarymage/cuauv/software/ASLAM/.cabal-sandbox/libexec"
sysconfdir = "/home/binarymage/cuauv/software/ASLAM/.cabal-sandbox/etc"

getBinDir, getLibDir, getDataDir, getLibexecDir, getSysconfDir :: IO FilePath
getBinDir = catchIO (getEnv "ASLAM_bindir") (\_ -> return bindir)
getLibDir = catchIO (getEnv "ASLAM_libdir") (\_ -> return libdir)
getDataDir = catchIO (getEnv "ASLAM_datadir") (\_ -> return datadir)
getLibexecDir = catchIO (getEnv "ASLAM_libexecdir") (\_ -> return libexecdir)
getSysconfDir = catchIO (getEnv "ASLAM_sysconfdir") (\_ -> return sysconfdir)

getDataFileName :: FilePath -> IO FilePath
getDataFileName name = do
  dir <- getDataDir
  return (dir ++ "/" ++ name)
