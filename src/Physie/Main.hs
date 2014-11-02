{-# LANGUAGE RankNTypes      #-}
{-# LANGUAGE TemplateHaskell #-}
module Main where

import           Control.Applicative    ((<$>), (<*>))
import           Control.Lens           (both, over, (^.))
import           Control.Lens.TH        (makeLenses)
import           Control.Monad          (msum, unless)
import           Control.Monad.Loops    (unfoldM)
import           Data.Monoid            ((<>))
import           Debug.Trace            (trace)
import           Graphics.UI.SDL.Events (pollEvent)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Types  as SDLT
import           Linear.Matrix          ((!*))
import           Linear.Metric          (dot, signorm)
import           Linear.V2              (V2 (..))
import           Linear.V3              (V3 (..))
import           Linear.Vector          ((*^))
import           Physie.ContactPoints   (findContactPoints)
import           Physie.Line
import           Physie.Sat             (satIntersects)
import           Physie.SDL             (drawLine, isQuitEvent, withImgInit,
                                         withRenderer, withWindow)
import           Physie.Time            (TimeTicks, getTicks, tickDelta,TimeDelta,fromNanoSeconds)

traceShowId :: forall a. Show a => String -> a -> a
traceShowId prefix a = trace (prefix <> show a) a

toIntPoint :: (RealFrac a,Integral b) => V2 a -> V2 b
toIntPoint (V2 x y) = V2 (floor x) (floor y)

minmax :: Ord a => [a] -> (a,a)
minmax ls = (minimum ls,maximum ls)

cross2 :: V2 Float -> V2 Float -> Float
cross2 (V2 x1 y1) (V2 x2 y2) = x1 * y2 - y1 * x2

lineIntersection :: Float -> Line -> Line -> Maybe (V2 Float)
lineIntersection delta (Line p to1) (Line q to2)
  | collinear && overlapping = Just (p + ((tnom/denom) *^ r))
  | collinear && not overlapping = Nothing
  | abs denom <= delta && abs unom > delta = Nothing
  | abs denom > delta && isNormalized (tnom / denom) && isNormalized (unom / denom) = Just (p + (tnom/denom) *^ r)
  | otherwise = Nothing
  where r = to1 - p
        s = to2 - q
        denom = r `cross2` s
        tnom = (q - p) `cross2` s
        unom = (q - p) `cross2` r
        isNormalized z = z >= 0 && z <= 1
        collinear = abs denom <= delta && abs unom <= delta
        overlapping = (0 <= ((q - p) `dot` r) && ((q - p) `dot` r) <= (r `dot` r)) || (0 <= (p - q) `dot` s && (p - q) `dot` s <= s `dot` s)

extractMaybe :: (Maybe a,b) -> Maybe (a,b)
extractMaybe (Nothing,_) = Nothing
extractMaybe (Just a,b) = Just (a,b)

data Rectangle = Rectangle Float Float

data RigidBody = RigidBody {
    _bodyPosition        :: V2 Float
  , _bodyRotation        :: Float
  , _bodyLinearVelocity  :: V2 Float
  , _bodyAngularVelocity :: Float
  , _bodyLinearForce     :: V2 Float
  , _bodyTorque          :: V2 Float
  , _bodyMass            :: Maybe Float
  , _bodyShape           :: Rectangle
  }

$(makeLenses ''RigidBody)

data Collision = Collision {
    _collContactPoint :: V2 Float
  , _collNormal       :: V2 Float
  } deriving(Show)

$(makeLenses ''Collision)

rectanglePoints :: V2 Float -> Float -> Rectangle -> [V2 Float]
rectanglePoints (V2 x y) rot (Rectangle w h) = (to2 . (rmatrix !*) . to3) <$> points
  where hw = w/2
        hh = h/2
        points = reverse [V2 (x-hw) (y-hh),V2 (x-hw) (y+hh),V2 (x+hw) (y+hh),V2 (x+hw) (y-hh)]
        r00 = cos rot
        r01 = -(sin rot)
        r10 = sin rot
        r11 = cos rot
        rmatrix = V3 (V3 r00 r01 (x-r00 * x - r01 * y)) (V3 r10 r11 (y - r10 * x - r11 * y)) (V3 0 0 1)
        to3 (V2 x' y') = V3 x' y' 1
        to2 (V3 x' y' _) = V2 x' y'

rectangleLines :: V2 Float -> Float -> Rectangle -> [Line]
rectangleLines pos rot rect = zipWith Line <*> (tail . cycle) $ rectanglePoints pos rot rect

detectCollision :: RigidBody -> RigidBody -> Maybe Collision
detectCollision b1 b2 = msum $ ((uncurry Collision <$>) . extractMaybe) <$> [(lineIntersection 0.001 x y,lineNormal x) | x <- bodyLines b1,y <- bodyLines b2]

bodyLines :: RigidBody -> [Line]
bodyLines b1 = rectangleLines (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)

bodyPoints :: RigidBody -> [V2 Float]
bodyPoints b1 = rectanglePoints (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

drawBody :: SDLT.Renderer -> RigidBody -> IO ()
drawBody r b = mapM_ (drawLine r) ((\(Line x y) -> (floor <$> x,floor <$> y)) <$> bodyLines b)

satIntersectsBodies :: RigidBody -> RigidBody -> Maybe (V2 Float)
satIntersectsBodies a b = case satIntersects (bodyLines a) (bodyLines b) of
  Nothing -> Nothing
  Just p -> Just $ if ((a ^. bodyPosition) - (b ^. bodyPosition)) `dot` p < 0
                   then negate p
                   else p

findSupportPoints :: V2 Float -> [V2 Float] -> [V2 Float]
findSupportPoints n vs = let dots = (`dot` n) <$> vs
                             mindot = minimum dots
                         in take 2 . map snd . filter (\(d,_) -> d < mindot + 0.001) $ zip dots vs

mainLoop :: SDLT.Renderer -> Float -> TimeTicks -> IO ()
mainLoop renderer angle oldticks = do
  newticks <- getTicks
  events <- unfoldM pollEvent
  let delta = newticks `tickDelta` oldticks
  let quitEvent = any isQuitEvent events
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  --let body1 = RigidBody {(V2 100 100) 0 (V2 0 0) 0 Nothing (Rectangle 100 100)
  let body1 = RigidBody {
      _bodyPosition = V2 100 100
    , _bodyRotation = 0
    , _bodyLinearVelocity = V2 0 0
    , _bodyAngularVelocity = 0
    , _bodyLinearForce = V2 0 0
    , _bodyTorque = V2 0 0
    , _bodyMass = Nothing
    , _bodyShape = Rectangle 100 100
  }
  let body2 = RigidBody {
      _bodyPosition = V2 100 200
    , _bodyRotation = 0.5
    , _bodyLinearVelocity = V2 0 0
    , _bodyAngularVelocity = 0
    , _bodyLinearForce = V2 0 0
    , _bodyTorque = V2 0 0
    , _bodyMass = Nothing
    , _bodyShape = Rectangle 100 100
  }
  drawBody renderer body1
  drawBody renderer body2
  SDLR.renderPresent renderer
  unless quitEvent $ mainLoop renderer (angle + 0.001) newticks
  {-
  -- Ergibt Vektor, der von body2 wegzeigt (damit sie nicht mehr kollidieren)
  let nraw = satIntersectsBodies body1 body2
  case nraw of
   Just nunpacked -> do
     let n1 = nunpacked
     putStrLn $ "nraw=" <> show (signorm nunpacked)
     -- Kontaktpunkt auf body2, Vektor zeigt von body2 weg (siehe oben)
     let cp = findContactPoints (bodyPoints body2) (bodyPoints body1) n1
     print cp
     SDLR.setRenderDrawColor renderer 255 0 0 255
     mapM_ (drawLine renderer . over both toIntPoint . (\p -> (p,p + 10 *^ n1))) cp
     SDLR.renderPresent renderer
     unless (any isQuitEvent events) $ mainLoop renderer (angle + 0.001)
   Nothing -> do
     SDLR.renderPresent renderer
     unless (any isQuitEvent events) $ mainLoop renderer (angle + 0.001)
     -}

maxDelta :: TimeDelta
maxDelta = fromNanoSeconds $ 1000 * 1000 * 10

splitDelta :: TimeDelta -> (Int,TimeDelta)
splitDelta n = undefined

simulationStep :: TimeDelta -> [RigidBody] -> [RigidBody]
simulationStep = undefined

main :: IO ()
main = do
  putStrLn "OLD (12,5) (8,5)"
  print $ findContactPoints [V2 8 4,V2 14 4,V2 14 9,V2 8 14] [V2 4 2,V2 12 2,V2 12 5,V2 4 5] (V2 0 (-1))
  putStrLn "NEW (6,4)"
  print $ findContactPoints (reverse [V2 2 8,V2 5 11,V2 9 7,V2 6 4]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 0 (-1))
  putStrLn "NEW 2 (12,5) (9.28,5)"
  print $ findContactPoints (reverse [V2 9 4,V2 10 8,V2 14 7,V2 13 3]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 (-0.19) (-0.98))

  withImgInit $
    withWindow "racie 0.0.1.1" $ \window -> do
      currentTicks <- getTicks
      withRenderer window screenWidth screenHeight $ \renderer -> mainLoop renderer 0 currentTicks
