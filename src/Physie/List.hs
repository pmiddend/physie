module Physie.List(
    maximumByNeighbors
  , boolToList
  ) where

import           Control.Lens  (view, _2)
import           Data.Function (on)
import           Data.List     (maximumBy)

maximumByNeighbors :: Ord a => (a -> a -> Ordering) -> [a] -> (a,a,a)
maximumByNeighbors f ls = let cls = cycle ls
                          in maximumBy (f `on` view _2) $ zip3 (drop (length ls - 1) cls) ls (drop 1 cls)

boolToList :: Bool -> a -> [a]
boolToList b a = [a | b]
