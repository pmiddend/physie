module Physie.ContactPoints(
  findContactPoints
  ) where

import           Control.Lens  ((^.))
import           Data.Monoid   ((<>))
import           Data.Ord      (comparing)
import           Debug.Trace   (trace)
import           Linear.Metric (dot, signorm,Metric)
import           Linear.V2     (V2 (..), _x, _y)
import           Linear.Vector ((*^))
import           Physie.Line
import           Physie.List   (boolToList, maximumByNeighbors)

traceShowId :: Show a => String -> a -> a
traceShowId prefix a = trace (prefix <> show a) a

findBestEdge :: (Num a, Num (f a), Ord (f a), Ord a, Metric f) => [f a] -> f a -> (f a, Line (f a))
findBestEdge ls n = let (v0,v,v1) = maximumByNeighbors (comparing (n `dot`)) ls
                    in if (v - v0) `dot` n <= (v - v1) `dot` n
                       then (v,Line v0 v)
                       else (v,Line v v1)

clip :: (Fractional a, Num (f a), Ord a, Metric f) => f a -> f a -> f a -> a -> [f a]
clip v1 v2 n o = let d1 = n `dot` v1 - o
                     d2 = n `dot` v2 - o
                     e = v2 - v1
                 in boolToList (d1 >= 0) v1 <>
                    boolToList (d2 >= 0) v2 <>
                    boolToList (d1 * d2 < 0) (v1 + (d1 / (d1 - d2)) *^ e)

findContactPoints :: (Fractional a, Ord a, Show a) => [V2 a] -> [V2 a] -> V2 a -> [V2 a]
findContactPoints a b n =
  let e1 = traceShowId "best edge 1: " $ findBestEdge a n
      e2 = traceShowId "best edge 2: " $ findBestEdge b (-n)
      e1Smaller = traceShowId "e1Smaller: " $ abs (lineVector (snd e1) `dot` n) <= abs (lineVector (snd e2) `dot` n)
      ref = if e1Smaller then e1 else e2
      inc = if e1Smaller then e2 else e1
      --nref = (signorm . lineVector . snd) ref
      nref = (lineVector . snd) ref
      o1 = nref `dot` lineStart (snd ref)
      [cp0,cp1] = clip (lineStart (snd inc)) (lineEnd (snd inc)) nref o1
      o2 = nref `dot` lineEnd (snd ref)
      [cp2,cp3] = clip cp0 cp1 (negate nref) (-o2)
      refNorm = (if e1Smaller then 1 else -1) *^ V2 (negate $ nref ^. _y) (nref ^. _x)
      refNormMax = refNorm `dot` fst ref
  in  boolToList (refNorm `dot` cp2 - refNormMax >= 0) cp2 <>
      boolToList (refNorm `dot` cp3 - refNormMax >= 0) cp3
