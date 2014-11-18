module Physie.SDL(
    withImgInit
  , withWindow
  , withRenderer
  , drawLines
  , drawLine
  , isQuitEvent
  , isKeydownEvent
  , withFontInit
  , createFontTexture
  , destroyTexture
  , sizeText
  ) where

import           Control.Applicative       ((<$>))
import           Control.Exception         (bracket, bracket_)
import           Data.Vector.Storable      (fromList)
import           Graphics.UI.SDL.Events    (Event (..), EventData (..))
import           Graphics.UI.SDL.Image     as SDLImage
import           Graphics.UI.SDL.Keysym    (Keysym (..), Scancode (Escape))
import qualified Graphics.UI.SDL.Rect      as SDLRect
import qualified Graphics.UI.SDL.Render    as SDLR
import qualified Graphics.UI.SDL.TTF       as SDLTtf
import           Graphics.UI.SDL.TTF.Types (TTFFont)
import qualified Graphics.UI.SDL.Types     as SDLT
import qualified Graphics.UI.SDL.Video     as SDLV
import           Linear.V2                 (V2 (..))
import           Graphics.UI.SDL.Color      (Color (..))
import           Control.Monad.Trans.Class  (MonadTrans, lift)

createFontTexture ::  (Monad (t IO), MonadTrans t) => SDLT.Renderer -> TTFFont -> String -> Color -> t IO SDLT.Texture
createFontTexture rend f text color = do
  surface <- lift $ SDLTtf.renderUTF8Blended f text color
  lift $ SDLR.createTextureFromSurface rend surface

destroyTexture ::  MonadTrans t => SDLT.Texture -> t IO ()
destroyTexture t = lift $ SDLR.destroyTexture t

sizeText :: (Monad (t IO),Functor (t IO), MonadTrans t) => TTFFont -> String -> t IO (V2 Int)
sizeText font text = uncurry V2 <$> lift (SDLTtf.sizeText font text)

withFontInit :: IO a -> IO a
withFontInit = bracket_ SDLTtf.init SDLTtf.quit

withImgInit :: IO a -> IO a
withImgInit = bracket_ (SDLImage.init [SDLImage.initPng]) SDLImage.quit

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

drawLine :: SDLT.Renderer -> (V2 Int,V2 Int) -> IO ()
drawLine renderer (V2 x1 y1,V2 x2 y2) = SDLR.renderDrawLine renderer (fromIntegral x1) (fromIntegral y1) (fromIntegral x2) (fromIntegral y2)

drawLines :: SDLT.Renderer -> [V2 Int] -> IO ()
drawLines renderer ls = SDLR.renderDrawLines renderer (fromList $ (\(V2 x y) -> SDLRect.Point x y) <$> ls)

isKeydownEvent :: Event -> Scancode -> Bool
isKeydownEvent (Event _ (Keyboard _ _ _ (Keysym sc _ _))) esc = sc == esc
isKeydownEvent _ _ = False

isQuitEvent :: Event -> Bool
isQuitEvent (Event _ Quit) = True
isQuitEvent (Event _ (Keyboard _ _ _ (Keysym sc _ _))) = sc == Escape
isQuitEvent _ = False
