module Physie.Sat(
  satIntersects
  ) where

import           Data.Function (on)
import           Data.List     (minimumBy)
import           Data.Maybe    (catMaybes, isNothing)
import           Linear.Metric (dot, quadrance)
import           Linear.V2     (V2 (..), perp)
import           Linear.Vector ((^*))
import           Physie.Line   (Line (..),lineStart,lineVector)
import           Physie.Ord    (minmax)
import Physie.Debug(traceShowId)

-- Courtesy of http://elancev.name/oliver/2D%20polygon.htm
satIntersects :: (Fractional b, Ord b,Show b) => [Line (V2 b)] -> [Line (V2 b)] -> Maybe (V2 b)
satIntersects a b = let axes = concatMap (map (perp . lineVector)) [a,b]
                        separationData = map (separates a b) axes
                    in if any isNothing separationData
                       then Nothing
                       else Just $ minimumBy (compare `on` quadrance) $ catMaybes separationData
  where calculateInterval axis = minmax . map ((axis `dot`) . lineStart)
        separates x y axis = let (mina,maxa) = calculateInterval axis x
                                 (minb,maxb) = calculateInterval axis y
                             in if mina > maxb || minb > maxa
                                then Nothing
                                else let d0 = maxa - minb
                                         d1 = maxb - mina
                                         depth = min d0 d1
                                     in Just $ axis ^* (depth / quadrance axis)
