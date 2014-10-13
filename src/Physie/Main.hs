module Main where

import Physie.SDL(withImgInit,withWindow,withRenderer,drawLines,isQuitEvent)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Types as SDLT
import           Graphics.UI.SDL.Events     (pollEvent)
import Control.Monad.Loops(unfoldM)
import Linear.V2(V2(..))

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

mainLoop :: SDLT.Renderer -> IO ()
mainLoop renderer = do
  events <- unfoldM pollEvent
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  drawLines renderer [V2 100 100,V2 100 200]
  SDLR.renderPresent renderer
  if any isQuitEvent events
     then return ()
     else mainLoop renderer

main :: IO ()
main = do
    withImgInit $ do
      withWindow "racie 0.0.1.1" $ \window -> do
        withRenderer window screenWidth screenHeight mainLoop
