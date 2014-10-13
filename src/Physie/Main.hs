module Main where

import Physie.SDL(withImgInit,withWindow,withRenderer,drawLines)
import qualified Graphics.UI.SDL.Render as SDLR
import Linear.V2(V2(..))
import Control.Monad(forever)

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

main :: IO ()
main = do
    withImgInit $ do
      withWindow "racie 0.0.1.1" $ \window -> do
        withRenderer window screenWidth screenHeight $ \renderer -> forever $ do
          SDLR.setRenderDrawColor renderer 0 0 0 255
          SDLR.renderClear renderer
          SDLR.setRenderDrawColor renderer 255 255 255 255
          drawLines renderer [V2 100 100,V2 100 200]
          SDLR.renderPresent renderer
