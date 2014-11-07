{-# LANGUAGE RankNTypes #-}
module Physie.Debug(
  traceShowId
  ) where

import Debug.Trace(trace)
import Data.Monoid((<>))

traceShowId :: forall a. Show a => String -> a -> a
traceShowId prefix a = trace (prefix <> show a) a


