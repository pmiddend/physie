module Physie.Colors(
    colorsWhite
  , colorsBlack
  , colorsRed
  ) where

import           Graphics.UI.SDL.Color      (Color (..))

colorsBlack :: Color
colorsBlack = Color 0 0 0 255

colorsWhite :: Color
colorsWhite = Color 255 255 255 255

colorsRed :: Color
colorsRed = Color 255 0 0 255

