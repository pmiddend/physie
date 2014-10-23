module Physie.Ord(
    clamp
  , minmax
  ) where

clamp :: Ord a => a -> a -> a -> a
clamp minv maxv = min maxv . max minv

minmax :: Ord a => [a] -> (a,a)
minmax ls = (minimum ls,maximum ls)
