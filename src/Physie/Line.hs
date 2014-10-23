module Physie.Line(
    Line(..)
  , lineVector
  , lineStart
  , lineEnd
  , lineNormal
  ) where

import           Linear.Metric (signorm)
import           Linear.V2     (V2, perp)

data Line = Line (V2 Float) (V2 Float) deriving Show

lineVector :: Line -> V2 Float
lineVector (Line a b) = b - a

lineStart :: Line -> V2 Float
lineStart (Line a _) = a

lineEnd :: Line -> V2 Float
lineEnd (Line _ b) = b

lineNormal :: Line -> V2 Float
lineNormal (Line a b) = perp .  signorm $ b - a
