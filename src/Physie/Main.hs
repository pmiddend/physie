{-# LANGUAGE TemplateHaskell #-}
module Main where

import Physie.SDL(withImgInit,withWindow,withRenderer,drawLine,isQuitEvent)
import qualified Graphics.UI.SDL.Render as SDLR
import qualified Graphics.UI.SDL.Types as SDLT
import           Graphics.UI.SDL.Events     (pollEvent)
import Control.Monad.Loops(unfoldM)
import Control.Lens((^.))
import Linear.V2(V2(..),perp)
import Linear.V3(V3(..))
import Linear.Matrix((!*),inv33)
import Linear.Vector((*^),(^*))
import Linear.Metric(dot,signorm,quadrance)
import Control.Lens.TH(makeLenses)
import Control.Monad(msum)
import Control.Applicative((<$>),(<*>))
import Data.Maybe(fromJust)
import Data.List(minimumBy)
import Data.Function(on)

data Rectangle = Rectangle Float Float

data RigidBody = RigidBody {
    _bodyPosition :: V2 Float
  , _bodyRotation :: Float
  , _bodyLinearVelocity :: V2 Float
  , _bodyAngularVelocity :: Float
  , _bodyMass :: Maybe Float
  , _bodyShape :: Rectangle
  }

$(makeLenses ''RigidBody)

data Collision = Collision {
    _collContactPoint :: V2 Float
  , _collNormal :: V2 Float
  } deriving(Show)

$(makeLenses ''Collision)

data Line = Line (V2 Float) (V2 Float)

rectangleLines :: V2 Float -> Float -> Rectangle -> [Line]
rectangleLines (V2 x y) rot (Rectangle w h) = zipWith Line <*> (tail . cycle) $ rpoints
  where hw = w/2
        hh = h/2
        points = [V2 (x-hw) (y-hh),V2 (x-hw) (y+hh),V2 (x+hw) (y+hh),V2 (x+hw) (y-hh)]
        r00 = cos rot
        r01 = -(sin rot)
        r10 = sin rot
        r11 = (cos rot)
        rmatrix = V3 (V3 r00 r01 (x-r00 * x - r01 * y)) (V3 r10 r11 (y - r10 * x - r11 * y)) (V3 0 0 1)
        to3 (V2 x' y') = V3 x' y' 1
        to2 (V3 x' y' _) = V2 x' y'
        rpoints = (to2 . (rmatrix !*) . to3) <$> points

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

lineNormal :: Line -> V2 Float
lineNormal (Line a b) = perp .  signorm $ b - a

detectCollision :: RigidBody -> RigidBody -> Maybe Collision
detectCollision b1 b2 = msum $ (((uncurry Collision) <$>) . extractMaybe) <$> [(lineIntersection 0.001 x y,lineNormal x) | x <- rectangleLines (b1 ^. bodyPosition) (b1 ^. bodyRotation) (b1 ^. bodyShape),y <- rectangleLines (b2 ^. bodyPosition) (b2 ^. bodyRotation) (b2 ^. bodyShape)]

screenWidth :: Int
screenWidth = 640

screenHeight :: Int
screenHeight = 480

drawBody :: SDLT.Renderer -> RigidBody -> IO ()
drawBody r b = mapM_ (drawLine r) ((\(Line x y) -> (floor <$> x,floor <$> y)) <$> (rectangleLines (b ^. bodyPosition) (b ^. bodyRotation) (b ^. bodyShape)))

toIntPoint (V2 x y) = V2 (floor x) (floor y)

minmax :: Ord a => [a] -> (a,a)
minmax ls = (minimum ls,maximum ls)

-- Courtesy of http://elancev.name/oliver/2D%20polygon.htm
data AxisSeparation = Separates | Overlaps (V2 Float) deriving Eq

satIntersects :: [Line] -> [Line] -> Maybe (V2 Float)
satIntersects a b = let axes = concatMap (map (perp . lineToVector)) [a,b]
                        separationData = map (separates a b) axes
                    in if any (== Separates) separationData
                       then Nothing
                       else Just $ minimumBy (compare `on` quadrance) $ concatMap (\x -> case x of Separates -> []; Overlaps x -> [x]) separationData
  where lineToVector (Line x y) = y - x
        calculateInterval axis = minmax . map ((axis `dot`) . lineToVector)
        separates x y axis = let (mina,maxa) = calculateInterval axis x
                                 (minb,maxb) = calculateInterval axis y
                             in if mina > maxb || minb > maxa
                                then Separates
                                else let d0 = maxa - minb
                                         d1 = maxb - mina
                                         depth = if d0 < d1 then d0 else d1
                                     in Overlaps $ axis ^* (depth / (quadrance axis))

mainLoop :: SDLT.Renderer -> Float -> IO ()
mainLoop renderer angle = do
  events <- unfoldM pollEvent
  SDLR.setRenderDrawColor renderer 0 0 0 255
  SDLR.renderClear renderer
  SDLR.setRenderDrawColor renderer 255 255 255 255
  let body1 = RigidBody (V2 100 100) 0.1 (V2 0 0) 0 Nothing (Rectangle 100 100)
      body2 = RigidBody (V2 150 150) 2.5 (V2 0 0) 0 Nothing (Rectangle 50 50)
  drawBody renderer body1
  drawBody renderer body2
  let collision = detectCollision body1 body2
  SDLR.setRenderDrawColor renderer 255 0 0 255
  case collision of
   Nothing -> return ()
   Just (Collision contactPoint normal) ->
     drawLine renderer (toIntPoint contactPoint,toIntPoint $ contactPoint + (normal ^* 10))
  SDLR.renderPresent renderer
  if any isQuitEvent events
     then return ()
     else mainLoop renderer (angle + 0.001)

main :: IO ()
main = do
    print $ detectCollision (RigidBody (V2 0 0) 0.1 (V2 0 0) 0 Nothing (Rectangle 10 10)) (RigidBody (V2 5 5) 0 (V2 0 0) 0 Nothing (Rectangle 5 5))
    withImgInit $ do
      withWindow "racie 0.0.1.1" $ \window -> do
        withRenderer window screenWidth screenHeight (\r -> mainLoop r 0)
