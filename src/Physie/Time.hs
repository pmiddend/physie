{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE RankNTypes #-}
module Physie.Time(
    TimeDelta
  , TimeTicks
  , tickDelta
  , getTicks
  , nanoSeconds
  , seconds
  , milliSeconds
) where

import           Data.Word           (Word64)
import           System.Clock        (Clock (Monotonic), TimeSpec (TimeSpec),
                                      getTime)

newtype TimeDelta = TimeDelta { _timeDelta :: Double } deriving(Show,Num,Eq,Ord)
newtype TimeTicks = TimeTicks { _timeTicks :: Word64 } deriving(Show,Num,Eq,Ord)

type TimeAccessor = Integral a => a -> TimeTicks

seconds :: Integral a => a -> TimeTicks
seconds = TimeTicks . fromIntegral . (* 1000000000)

milliSeconds = TimeTicks . fromIntegral . (* 1000000)

nanoSeconds :: Int -> TimeTicks
nanoSeconds = TimeTicks . fromIntegral

tickDelta :: TimeTicks -> TimeTicks -> TimeDelta
tickDelta new old = TimeDelta $ fromIntegral (_timeTicks new - _timeTicks old) / (1000.0 * 1000.0 * 1000.0)

getTicks :: IO TimeTicks
getTicks = do
  (TimeSpec s ns) <- getTime Monotonic
  return $ seconds s + nanoSeconds ns
