{-# LANGUAGE RankNTypes      #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE TypeFamilies #-}
module Main where

import           Control.Applicative    ((<$>), (<*>))
import           Control.Lens           (IndexPreservingGetter, ix, to, (&),
                                         (*~),(+~), (.~), (^.), (-~),(^?!))
import           Control.Lens.At        (Index, Ixed,IxValue)
import           Control.Lens.TH        (makeLenses)
import           Control.Monad          (msum, unless)
import           Control.Monad.Loops    (unfoldM)
import           Data.Foldable          (foldMap)
import           Data.List              (tails)
import           Data.Maybe             (fromJust, isJust,mapMaybe)
import           Data.Monoid            (Endo (Endo), appEndo, (<>))
import           Debug.Trace            (trace)
import           Graphics.UI.SDL.Events (pollEvent)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Types  as SDLT
import           Linear.Matrix          ((!*))
import           Linear.Metric          (dot)
import           Linear.V2              (V2 (..))
import           Linear.V3              (V3 (..),cross,_z)
import           Linear.Vector          ((*^), (^*), (^/))
import           Physie.ContactPoints   (findContactPoints)
import           Physie.Line
import           Physie.Sat             (satIntersects)
import           Physie.SDL             (drawLine, isQuitEvent, withImgInit,
                                         withRenderer, withWindow)
import           Physie.Time            (TimeDelta, TimeTicks, fromSeconds,
                                         getTicks, tickDelta, toSeconds)
import Physie.Debug(traceShowId)

toIntPoint :: (RealFrac a,Integral b) => V2 a -> V2 b
toIntPoint (V2 x y) = V2 (floor x) (floor y)

minmax :: Ord a => [a] -> (a,a)
minmax ls = (minimum ls,maximum ls)

cross2 :: V2 Double -> V2 Double -> Double
cross2 (V2 x1 y1) (V2 x2 y2) = x1 * y2 - y1 * x2

lineIntersection :: Double -> Line (V2 Double) -> Line (V2 Double) -> Maybe (V2 Double)
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

extractMaybe1of2 :: (Maybe a,b) -> Maybe (a,b)
extractMaybe1of2 (Nothing,_) = Nothing
extractMaybe1of2 (Just a,b) = Just (a,b)

extractMaybe3of3 :: (a,b,Maybe c) -> Maybe (a,b,c)
extractMaybe3of3 (_,_,Nothing) = Nothing
extractMaybe3of3 (a,b,Just c) = Just (a,b,c)

data Rectangle = Rectangle Double Double deriving(Show)

rectangleInertia :: Double -> Rectangle -> Double
rectangleInertia m (Rectangle w h) = 1.0/12.0 * m * (w*w + h*h)

data RigidBody = RigidBody {
    _bodyPosition        :: V2 Double
  , _bodyRotation        :: Double
  , _bodyLinearVelocity  :: V2 Double
  , _bodyAngularVelocity :: Double
  , _bodyLinearForce     :: V2 Double
  , _bodyTorque          :: V2 Double
  , _bodyMass            :: Maybe Double
  , _bodyShape           :: Rectangle
  } deriving(Show)

$(makeLenses ''RigidBody)

bodyInertia :: IndexPreservingGetter RigidBody (Maybe Double)
bodyInertia = to $ \b -> (\m -> rectangleInertia m (b ^. bodyShape)) <$> (b ^. bodyMass)

data Collision = Collision {
    _collContactPoints :: [V2 Double]
  , _collNormal        :: V2 Double
  } deriving(Show)

$(makeLenses ''Collision)

rectanglePoints :: V2 Double -> Double -> Rectangle -> [V2 Double]
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

rectangleLines :: V2 Double -> Double -> Rectangle -> [Line (V2 Double)]
rectangleLines pos rot rect = zipWith Line <*> (tail . cycle) $ rectanglePoints pos rot rect

detectCollision :: RigidBody -> RigidBody -> Maybe Collision
detectCollision b1 b2 = msum $ ((uncurry Collision <$>) . extractMaybe1of2) <$> [(fmap return $ lineIntersection 0.001 x y,lineNormal x) | x <- bodyLines b1,y <- bodyLines b2]

bodyLines :: RigidBody -> [Line (V2 Double)]
bodyLines b1 = rectangleLines (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)

-- bodyPoints :: RigidBody -> [V2 Double]
-- bodyPoints b1 = rectanglePoints (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)
bodyPoints :: IndexPreservingGetter RigidBody [V2 Double]
bodyPoints = to $ \b1 -> rectanglePoints (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape)

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

drawBody :: SDLT.Renderer -> RigidBody -> IO ()
drawBody r b = mapM_ (drawLine r) ((\(Line x y) -> (floor <$> x,floor <$> y)) <$> bodyLines b)

satIntersectsBodies :: RigidBody -> RigidBody -> Maybe (V2 Double)
satIntersectsBodies a b = case satIntersects (bodyLines a) (bodyLines b) of
  Nothing -> Nothing
  Just p -> Just $ if ((a ^. bodyPosition) - (b ^. bodyPosition)) `dot` p < 0
                   then negate p
                   else p

findSupportPoints :: V2 Double -> [V2 Double] -> [V2 Double]
findSupportPoints n vs = let dots = (`dot` n) <$> vs
                             mindot = minimum dots
                         in take 2 . map snd . filter (\(d,_) -> d < mindot + 0.001) $ zip dots vs

mainLoop :: SDLT.Renderer -> TimeTicks -> TimeDelta -> [RigidBody] -> IO ()
mainLoop renderer oldticks oldDelta bodies = do
  events <- unfoldM pollEvent
  let quitEvent = any isQuitEvent events
  newticks <- getTicks
  let delta = newticks `tickDelta` oldticks
  let (iterations,newDelta) = splitDelta (oldDelta + delta)
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  --let body1 = RigidBody {(V2 100 100) 0 (V2 0 0) 0 Nothing (Rectangle 100 100)
  {-
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
  -}
  mapM_ (drawBody renderer) bodies
  SDLR.renderPresent renderer
  unless quitEvent $ mainLoop renderer newticks newDelta (iterate (simulationStep maxDelta) bodies !! iterations)
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
maxDelta = fromSeconds 0.01

splitDelta :: TimeDelta -> (Int,TimeDelta)
splitDelta n = let iterations = floor $ toSeconds n / toSeconds maxDelta
               in (iterations,n - fromIntegral iterations * maxDelta)

updateBody :: TimeDelta -> RigidBody -> RigidBody
updateBody ds b | isJust (b ^. bodyMass) = let d = toSeconds ds
                                               la = ((b ^. bodyLinearForce) ^/ fromJust (b ^. bodyMass))
                                               lv = (b ^. bodyLinearVelocity) + la ^* d
                                               p = (b ^. bodyPosition) + lv ^* d
                                          in b {
                                                   _bodyLinearVelocity = lv
                                                 , _bodyPosition = p
                                                 , _bodyRotation = (b ^. bodyRotation) + d * (b ^. bodyAngularVelocity)
                                               }
               | otherwise = b

unorderedPairs :: forall b. [b] -> [(b, b)]
unorderedPairs input = let ts = tails input
                       in concat $ zipWith zip (map (cycle . take 1) ts) (drop 1 ts)

generateCollisionData :: RigidBody -> RigidBody -> Maybe Collision
generateCollisionData a b = satIntersectsBodies a b >>= helper
  where helper n = case findContactPoints (a ^. bodyPoints) (b ^. bodyPoints) n of
                    [] -> Nothing
                    xs -> Just $ Collision xs n

toV3 :: V2 a -> a -> V3 a
toV3 (V2 x y) = V3 x y

processCollision :: (Ixed c, IxValue c ~ RigidBody) => ((Index c, RigidBody), (Index c, RigidBody), Collision) -> c -> c
processCollision ((ixa,a),(ixb,b),colldata) = let e = 0.1
                                                  n = colldata ^. collNormal
                                                  cp = (traceShowId "contPoints: " $ colldata ^. collContactPoints) ^?! ix 0
                                                  va = a ^. bodyLinearVelocity
                                                  vb = b ^. bodyLinearVelocity
                                                  m1 = fromJust (a ^. bodyMass)
                                                  m2 = fromJust (b ^. bodyMass)
                                                  vab = va - vb
                                                  nom = (-(1+e) *^ vab) `dot` n
                                                  denomb = (n `dot` n) * (recip m1 + recip m2)
                                                  impulse = nom / denomb
                                                  ra = toV3 (cp - a ^. bodyPosition) 0
                                                  rb = toV3 (cp - b ^. bodyPosition) 0
                                                  n3 = toV3 n 0
                                                  jamdf b v = ((v `cross` n3) & _z *~ recip (fromJust $ b ^. bodyInertia)) `cross` v
                                                  jamd1 = jamdf a ra
                                                  jamd2 = jamdf b rb
                                                  jamd = denomb + (jamd1 + jamd2) `dot` n3
                                                  jam = nom / jamd
                                                  angularvf b v = jam * recip (fromJust $ b ^. bodyInertia) * (v `cross` n3) ^. _z
                                                  angularva = angularvf a ra
                                                  angularvb = angularvf b rb
                                                  newa = a & bodyLinearVelocity +~ (impulse / fromJust (a ^. bodyMass)) *^ n & bodyAngularVelocity +~ angularva
                                                  newb = a & bodyLinearVelocity +~ (impulse / fromJust (b ^. bodyMass)) *^ n & bodyAngularVelocity +~ angularvb
                                              in (ix ixa .~ newa) . (ix ixb .~ newb)

simulationStepSimple :: TimeDelta -> [RigidBody] -> [RigidBody]
simulationStepSimple d = map (updateBody d)

simulationStep :: TimeDelta -> [RigidBody] -> [RigidBody]
simulationStep d bs = let stepResult = map (updateBody d) bs
                          collisionResults = mapMaybe (extractMaybe3of3 . (\(l@(_,bl),r@(_,br)) -> (l,r,generateCollisionData bl br))) (unorderedPairs (zip ([0..] :: [Int]) stepResult))
                      in foldMap Endo (map processCollision collisionResults) `appEndo` stepResult

main :: IO ()
main = do
--   putStrLn "OLD (12,5) (8,5)"
--   print $ findContactPoints [V2 8 4,V2 14 4,V2 14 9,V2 8 14] [V2 4 2,V2 12 2,V2 12 5,V2 4 5] (V2 0 (-1))
--   putStrLn "NEW (6,4)"
--   print $ findContactPoints (reverse [V2 2 8,V2 5 11,V2 9 7,V2 6 4]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 0 (-1))
--   putStrLn "NEW 2 (12,5) (9.28,5)"
--   print $ findContactPoints (reverse [V2 9 4,V2 10 8,V2 14 7,V2 13 3]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 (-0.19) (-0.98))
  let initialBodies = [
               RigidBody {
              _bodyPosition = V2 100 100
            , _bodyRotation = 0
            , _bodyLinearVelocity = V2 10 0
            , _bodyAngularVelocity = 0
            , _bodyLinearForce = V2 100 0
            , _bodyTorque = V2 0 0
            , _bodyMass = Just 100
            , _bodyShape = Rectangle 100 100
            }, RigidBody {
              _bodyPosition = V2 300 150
            , _bodyRotation = 0.5
            , _bodyLinearVelocity = V2 0 0
            , _bodyAngularVelocity = 0
            , _bodyLinearForce = V2 0 0
            , _bodyTorque = V2 0 0
            , _bodyMass = Just 1
            , _bodyShape = Rectangle 100 100
            }
            ]

--   print $ satIntersectsBodies (initialBodies !! 0) (initialBodies !! 1)

  withImgInit $
    withWindow "racie 0.0.1.1" $ \window -> do
      currentTicks <- getTicks
      withRenderer window screenWidth screenHeight $ \renderer -> mainLoop renderer currentTicks (fromSeconds 0) initialBodies
