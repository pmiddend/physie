module Physie.Line(
    Line(..)
  , lineVector
  , lineStart
  , lineEnd
  , lineNormal
  ) where

import           Linear.Metric (signorm)
import           Linear.V2     (V2, perp)

-- FIXME: Lenses

data Line a = Line a a deriving Show

lineVector :: Num a => Line a -> a
lineVector (Line a b) = b - a

lineStart :: Line a -> a
lineStart (Line a _) = a

lineEnd :: Line t -> t
lineEnd (Line _ b) = b

lineNormal :: Floating a => Line (V2 a) -> V2 a
lineNormal = perp .  signorm . lineVector
