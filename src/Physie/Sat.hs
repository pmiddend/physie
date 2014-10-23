module Physie.Sat(
  satIntersects
  ) where

import           Data.Function (on)
import           Data.List     (minimumBy)
import           Data.Maybe    (catMaybes, isNothing)
import           Linear.Metric (dot, quadrance)
import           Linear.V2     (V2 (..), perp)
import           Linear.Vector ((^*))
import           Physie.Line   (Line (..))
import           Physie.Ord    (minmax)

-- Courtesy of http://elancev.name/oliver/2D%20polygon.htm
satIntersects :: [Line] -> [Line] -> Maybe (V2 Float)
satIntersects a b = let axes = concatMap (map (perp . lineToVector)) [a,b]
                        separationData = map (separates a b) axes
                    in if any isNothing separationData
                       then Nothing
                       else Just $ minimumBy (compare `on` quadrance) $ catMaybes separationData
  where lineToVector (Line x y) = y - x
        calculateInterval axis = minmax . map ((axis `dot`) . lineToVector)
        separates x y axis = let (mina,maxa) = calculateInterval axis x
                                 (minb,maxb) = calculateInterval axis y
                             in if mina > maxb || minb > maxa
                                then Nothing
                                else let d0 = maxa - minb
                                         d1 = maxb - mina
                                         depth = if d0 < d1 then d0 else d1
                                     in Just $ axis ^* (depth / quadrance axis)
