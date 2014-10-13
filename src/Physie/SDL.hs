module Physie.SDL(
    withImgInit
  , withWindow
  , withRenderer
  , drawLines
  , isQuitEvent
  ) where

import Control.Applicative((<$>))
import           Control.Exception      (bracket, bracket_)
import           Graphics.UI.SDL.Events     (Event (..), EventData (..))
import           Graphics.UI.SDL.Keysym     (Keysym (..),
                                             Scancode (Escape))
import           Graphics.UI.SDL.Image  as SDLImage
import Data.Vector.Storable(fromList)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Rect as SDLRect
import qualified Graphics.UI.SDL.Types  as SDLT
import qualified Graphics.UI.SDL.Video  as SDLV
import Linear.V2(V2(..))

withImgInit :: IO a -> IO a
withImgInit a = bracket_ (SDLImage.init [SDLImage.initPng]) SDLImage.quit a

screenAbsoluteWidth :: Int
screenAbsoluteWidth = 0

screenAbsoluteHeight :: Int
screenAbsoluteHeight = 0

windowFlags :: [SDLT.WindowFlag]
windowFlags = [SDLT.WindowResizable]

withWindow :: String -> (SDLT.Window -> IO a) -> IO a
withWindow title callback = SDLV.withWindow
                            title
                            (SDLT.Position SDLV.windowPosUndefined SDLV.windowPosUndefined)
                            (SDLT.Size (fromIntegral screenAbsoluteWidth) (fromIntegral screenAbsoluteHeight))
                            windowFlags $ \win -> callback win

withRenderer :: SDLT.Window -> Int -> Int -> (SDLT.Renderer -> IO a) -> IO a
withRenderer window screenWidth screenHeight callback =
  let acquireResource = SDLR.createRenderer window SDLT.FirstSupported []
      releaseResource = SDLR.destroyRenderer
  in bracket acquireResource releaseResource $ \renderer -> do
    SDLR.renderSetLogicalSize renderer (fromIntegral screenWidth) (fromIntegral screenHeight)
    callback renderer

drawLines :: SDLT.Renderer -> [V2 Int] -> IO ()
drawLines renderer ls = SDLR.renderDrawLines renderer (fromList $ (\(V2 x y) -> SDLRect.Point x y) <$> ls)

isQuitEvent :: Event -> Bool
isQuitEvent (Event _ Quit) = True
isQuitEvent (Event _ (Keyboard _ _ _ (Keysym sc _ _))) = sc == Escape
isQuitEvent _ = False
