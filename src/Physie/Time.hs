{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE RankNTypes                 #-}
module Physie.Time(
    TimeDelta
  , TimeTicks
  , tickDelta
  , getTicks
  , fromSeconds
  , toSeconds
) where

import           Data.Word    (Word64)
import           System.Clock (Clock (Monotonic), TimeSpec (TimeSpec), getTime)

newtype TimeDelta = TimeDelta { _timeDelta :: Double } deriving(Show,Num,Eq,Ord)
newtype TimeTicks = TimeTicks { _timeTicks :: Word64 } deriving(Show,Num,Eq,Ord)

tickSeconds :: Integral a => a -> TimeTicks
tickSeconds = TimeTicks . fromIntegral . (* 1000000000)

tickNanoSeconds :: Int -> TimeTicks
tickNanoSeconds = TimeTicks . fromIntegral

tickDelta :: TimeTicks -> TimeTicks -> TimeDelta
tickDelta new old = TimeDelta $ fromIntegral (_timeTicks new - _timeTicks old) / (1000.0 * 1000.0 * 1000.0)

fromSeconds :: Double -> TimeDelta
fromSeconds = TimeDelta

toSeconds :: TimeDelta -> Double
toSeconds (TimeDelta t) = t

getTicks :: IO TimeTicks
getTicks = do
  (TimeSpec s ns) <- getTime Monotonic
  return $ tickSeconds s + tickNanoSeconds ns
