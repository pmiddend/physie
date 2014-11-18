{-# LANGUAGE RankNTypes      #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE TypeFamilies    #-}
module Main where

import           Control.Applicative        ((<$>), (<*>))
import           Control.Lens               (IndexPreservingGetter, both, each,
                                             ix, over, to, use, view, (%=), (&),
                                             (*~), (+=), (+~), (-~), (.=), (.~),
                                             (^.), (^?!), _1, _2, _3)
import           Control.Lens.At            (Index, IxValue, Ixed, at)
import           Control.Lens.TH            (makeClassy, makeLenses)
import           Control.Monad              (msum, unless, void, when)
import           Control.Monad.Loops        (unfoldM)
import           Control.Monad.State.Strict (StateT, runStateT)
import           Control.Monad.Trans.Class  (MonadTrans, lift)
import           Data.Foldable              (foldMap)
import           Data.List                  (tails,maximum)
import           Data.Maybe                 (fromJust, isJust, mapMaybe)
import           Data.Monoid                (Endo (Endo), appEndo, (<>))
import           Data.Traversable           (traverse)
import           Data.Word                  (Word8)
import           Debug.Trace                (trace)
import           Graphics.UI.SDL.Color      (Color (..))
import           Graphics.UI.SDL.Events     (Event (..), pollEvent)
import           Graphics.UI.SDL.Keysym     (Scancode (LeftShift, LeftControl))
import qualified Graphics.UI.SDL.Rect       as SDLRect
import qualified Graphics.UI.SDL.Render     as SDLR
import qualified Graphics.UI.SDL.TTF        as SDLTtf
import           Graphics.UI.SDL.TTF.Types  (TTFFont)
import qualified Graphics.UI.SDL.Types      as SDLT
import           Linear.Matrix              ((!*))
import           Linear.Metric              (dot,distance)
import           Linear.V2                  (V2 (..), _x, _y)
import           Linear.V3                  (V3 (..), cross, _z)
import           Linear.Vector              ((*^), (^*), (^/))
import           Physie.Colors
import           Physie.ContactPoints       (findContactPoints)
import           Physie.Debug               (traceShowId)
import           Physie.Line
import           Physie.Sat                 (satIntersects)
import           Physie.SDL                 (createFontTexture, destroyTexture,
                                             drawLine, isKeydownEvent,
                                             isQuitEvent, withFontInit,
                                             withImgInit, withRenderer,
                                             withWindow,sizeText)
import           Physie.Time                (TimeDelta, TimeTicks, fromSeconds,
                                             getTicks, tickDelta, toSeconds)
import           System.FilePath

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
        --points = reverse [V2 (x-hw) (y-hh),V2 (x-hw) (y+hh),V2 (x+hw) (y+hh),V2 (x+hw) (y-hh)]
        -- FIXME
        points = [V2 (x-hw) (y-hh),V2 (x-hw) (y+hh),V2 (x+hw) (y+hh),V2 (x+hw) (y-hh)]
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

data ConsoleMessage = ConsoleMessage {
    _conMesText  :: String
  , _conMesStart :: TimeTicks
  } deriving Show

$(makeLenses ''ConsoleMessage)

data GameStateData = GameStateData {
    _renderer        :: SDLT.Renderer
  , _font            :: TTFFont
  , _ticks           :: TimeTicks
  , _prevDelta       :: TimeDelta
  , _bodies          :: [RigidBody]
  , _timeMultiplier  :: Double
  , _contactPointSum :: Int
  , _consoleMessages :: [ConsoleMessage]
  }

$(makeLenses ''GameStateData)

type GameState a = StateT GameStateData IO a

data TextPosition = LeftTop | LeftBottom | RightTop | RightBottom

trimConsoleMessages :: GameState ()
trimConsoleMessages = do
  t <- use ticks
  consoleMessages %= filter (\cm -> t `tickDelta` view conMesStart cm < fromSeconds 0.5)
  return ()

textSize :: String -> GameState (V2 Int)
textSize s = do
  f <- use font
  sizes <- mapM (sizeText f) (lines s)
  return $ V2 (maximum . map (view _x) $ sizes) (sum . map (view _y) $ sizes)

createAndRenderTextRelative :: String -> Color -> TextPosition -> GameState ()
createAndRenderTextRelative [] _ _ = return ()
createAndRenderTextRelative text color position = do
  (V2 width height) <- textSize text
  let realPosition = case position of
                        LeftTop -> V2 0 0
                        LeftBottom -> V2 0 (screenHeight - height)
                        RightTop -> V2 (screenWidth - width) 0
                        RightBottom -> V2 (screenWidth - width) (screenHeight - height)
  createAndRenderText text color (fromIntegral <$> realPosition)

feedbackFold :: Monad m => (a -> b -> m b) -> [a] -> b -> m ()
feedbackFold _ [] _ = return ()
feedbackFold f (x:xs) s = f x s >>= feedbackFold f xs

createAndRenderText :: String -> Color -> V2 Double -> GameState ()
createAndRenderText text color position = feedbackFold (createAndRenderSingleLine color) (lines text) (toIntPoint position)

createAndRenderSingleLine :: Color -> String -> V2 Int -> GameState (V2 Int)
createAndRenderSingleLine _ [] _ = do
  f <- use font
  h <- lift $ SDLTtf.getFontHeight f
  return $ V2 0 h
createAndRenderSingleLine color text position = do
  f <- use font
  rend <- use renderer
  texture <- createFontTexture rend f text color
  size <- sizeText f text
  lift $ SDLR.renderCopy rend texture Nothing (Just $ SDLRect.Rect (position ^. _x) (position ^. _y) (size ^. _x) (size ^. _y))
  destroyTexture texture
  return (V2 0 ((position + size) ^. _y))

drawBody :: RigidBody -> GameState ()
drawBody b = do
  r <- use renderer
  lift $ mapM_ (drawLine r) ((\(Line x y) -> (floor <$> x,floor <$> y)) <$> bodyLines b)

satIntersectsBodies :: RigidBody -> RigidBody -> Maybe (V2 Double)
satIntersectsBodies a b = case satIntersects (bodyLines a) (bodyLines b) of
  Nothing -> Nothing
  Just p -> Just $ if ((a ^. bodyPosition) - (b ^. bodyPosition)) `dot` p < 0
                   then p
                   else negate p

findSupportPoints :: V2 Double -> [V2 Double] -> [V2 Double]
findSupportPoints n vs = let dots = (`dot` n) <$> vs
                             mindot = minimum dots
                         in take 2 . map (view _2) . filter (\(d,_) -> d < mindot + 0.001) $ zip dots vs

drawPoint :: V2 Double -> GameState ()
drawPoint p = do
  r <- use renderer
  let o = 3
      ls = [
             (p + V2 (-o) (-o),p + V2 o (-o))
           , (p + V2 o (-o),p + V2 o o)
           , (p + V2 o o,p + V2 (-o) o)
           , (p + V2 (-o) o,p + V2 (-o) (-o))]
  lift $ mapM_ (drawLine r) $ over (traverse . each) toIntPoint ls

updateTimeMultiplier :: [Event] -> Double -> Double
updateTimeMultiplier es oldTimeMultiplier
  | any (`isKeydownEvent` LeftControl) es = max 0 $ oldTimeMultiplier - 0.1
  | any (`isKeydownEvent` LeftShift) es = oldTimeMultiplier + 0.1
  | otherwise = oldTimeMultiplier

sdlSetRenderDrawColor :: Color -> GameState ()
sdlSetRenderDrawColor (Color r g b a) = use renderer >>= \rend -> lift $ SDLR.setRenderDrawColor rend r g b a

sdlRenderClear :: GameState ()
sdlRenderClear = use renderer >>= \rend -> lift $ SDLR.renderClear rend

sdlRenderPresent :: GameState ()
sdlRenderPresent = use renderer >>= \rend -> lift $ SDLR.renderPresent rend

data SimulationStep = SimulationStep {
    _simStepCollPoints :: [V2 Double]
  , _simStepBodies     :: [RigidBody]
  }

$(makeLenses ''SimulationStep)

mainLoop :: GameState ()
mainLoop = do
  events <- unfoldM (lift pollEvent)
  oldTicks <- use ticks
  newTicks <- lift getTicks
  ticks .= newTicks
  let delta' = newTicks `tickDelta` oldTicks
  oldDelta <- use prevDelta
  oldtm <- use timeMultiplier
  timeMultiplier %= updateTimeMultiplier events
  newtm <- use timeMultiplier
  when (newtm /= oldtm) (consoleMessages %= ([ConsoleMessage ("time changed to " <> show newtm) newTicks] ++))
  let delta = fromSeconds $ newtm * toSeconds delta'
  let (iterations,newDelta) = splitDelta (oldDelta + delta)
  prevDelta .= newDelta
  sdlSetRenderDrawColor colorsBlack
  sdlRenderClear
  sdlSetRenderDrawColor colorsWhite
  oldBodies <- use bodies
  mapM_ drawBody oldBodies
  mapM_ (lift . print) oldBodies
  let simulationResult = iterate (simulationStep maxDelta . view simStepBodies) (SimulationStep [] oldBodies)
  bodies .= view simStepBodies (simulationResult !! iterations)
  let points = concatMap (view simStepCollPoints) (take (iterations+1) simulationResult)
  sdlSetRenderDrawColor colorsRed
  mapM_ drawPoint points
  contactPointSum += length points
  cpsum <- use contactPointSum
  createAndRenderTextRelative ("cps: " <> show cpsum) colorsWhite LeftTop
  cms <- use consoleMessages
  createAndRenderTextRelative (unlines . map (view conMesText) $ cms) colorsWhite LeftBottom
  trimConsoleMessages
  sdlRenderPresent
  unless (any isQuitEvent events) mainLoop

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
   --  where helper n = case findContactPoints (reverse (a ^. bodyPoints)) (reverse (b ^. bodyPoints)) ((-1) *^ n) of
   where helper n = case findContactPoints (a ^. bodyPoints) (b ^. bodyPoints) n of
                    [] -> Nothing
                    xs -> Just $ Collision xs n
--generateCollisionData a b = (\n -> Collision (findContactPoints (a ^. bodyPoints) (b ^. bodyPoints) n) n) <$> satIntersectsBodies a b

toV3 :: V2 a -> a -> V3 a
toV3 (V2 x y) = V3 x y

angularFunction :: V2 Double
                     -> RigidBody
                     -> RigidBody
                     -> V2 Double
                     -> Double
                     -> Double
                     -> (Double, Double)
angularFunction cp a b n nom denomb =
  let angularvf body v = jam * recip (fromJust $ body ^. bodyInertia) * (v `cross` n3) ^. _z
      jamdf body v = ((v `cross` n3) & _z *~ recip (fromJust $ body ^. bodyInertia)) `cross` v
      ra = toV3 (cp - a ^. bodyPosition) 0
      rb = toV3 (cp - b ^. bodyPosition) 0
      jamd1 = jamdf a ra
      jamd2 = jamdf b rb
      jamd = denomb + (jamd1 + jamd2) `dot` n3
      jam = nom / jamd
      n3 = toV3 n 0
  in (angularvf a ra,angularvf b rb)

processCollision :: (Ixed c, IxValue c ~ RigidBody) => ((Index c, RigidBody), (Index c, RigidBody), Collision) -> c -> c
processCollision ((ixa,a),(ixb,b),colldata) = let e = 0.1
                                                  cps = colldata ^. collContactPoints
                                                  n = colldata ^. collNormal
                                                  va = a ^. bodyLinearVelocity
                                                  vb = b ^. bodyLinearVelocity
                                                  vab = va - vb
                                                  ma = fromJust (a ^. bodyMass)
                                                  mb = fromJust (b ^. bodyMass)
                                                  nom = ((-1) * (1+e) * (vab `dot` n))
                                                  denomb = (n `dot` n) * (recip ma + recip mb)
                                                  imp = nom / denomb
                                                  (angularvas,angularvbs) = unzip . map (\cp -> angularFunction cp a b n nom denomb) $ cps
                                                  newa = a & bodyLinearVelocity +~ (imp / fromJust (a ^. bodyMass)) *^ n & bodyAngularVelocity +~ sum angularvas
                                                  newb = b & bodyLinearVelocity -~ (imp / fromJust (b ^. bodyMass)) *^ n & bodyAngularVelocity -~ sum angularvbs
                                              in (ix ixa .~ newa) . (ix ixb .~ newb)

simulationStepSimple :: TimeDelta -> [RigidBody] -> [RigidBody]
simulationStepSimple d = map (updateBody d)

simulationStep :: TimeDelta -> [RigidBody] -> SimulationStep
simulationStep d bs = let stepResult = map (updateBody d) bs
                          collisionResults = mapMaybe (extractMaybe3of3 . (\(l@(_,bl),r@(_,br)) -> (l,r,generateCollisionData bl br))) (unorderedPairs (zip ([0..] :: [Int]) stepResult))
                      in SimulationStep (concatMap (view (_3 . collContactPoints)) collisionResults) (foldMap Endo (map processCollision collisionResults) `appEndo` stepResult)

mediaDir :: FilePath
mediaDir = "media"

initialBodies :: [RigidBody]
initialBodies = [
               RigidBody {
              _bodyPosition = V2 100 100
            , _bodyRotation = 0
            , _bodyLinearVelocity = V2 10 0
            , _bodyAngularVelocity = 0
            , _bodyLinearForce = V2 100 0
            , _bodyTorque = V2 0 0
            , _bodyMass = Just 10
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

initialGameState :: SDLT.Renderer -> TTFFont -> TimeTicks -> GameStateData
initialGameState rend stdFont currentTicks = GameStateData rend stdFont currentTicks (fromSeconds 0) initialBodies 1 0 []

firstCpTest =
  let [cp1,cp2] = findContactPoints [V2 8 4,V2 14 4,V2 14 9,V2 8 14] [V2 4 2,V2 12 2,V2 12 5,V2 4 5] (V2 0 (-1))
  in if cp1 `distance` V2 12 5 > 0.1 || cp2 `distance` V2 8 5 > 0.1
        then putStrLn ("Error in test 1: " <> show cp1 <> ", " <> show cp2) >> return False
        else return True

secondCpTest =
  let [cp1] = findContactPoints (reverse [V2 2 8,V2 5 11,V2 9 7,V2 6 4]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 0 (-1))
  in if cp1 `distance` V2 6 4 > 0.1
        then putStrLn ("Error in test 2: " <> show cp1) >> return False
        else return True

thirdCpTest =
  let [cp1,cp2] = findContactPoints (reverse [V2 9 4,V2 10 8,V2 14 7,V2 13 3]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 (-0.19) (-0.98))
  in if cp1 `distance` V2 12 5 > 0.1 || cp2 `distance` V2 9.28 5 > 0.1
        then putStrLn ("Error in test 3: " <> show cp1 <> ", " <> show cp2) >> return False
        else return True

gcTest = let Just gd = generateCollisionData testBody1 testBody2
             testBody1 = RigidBody {
                              _bodyPosition = V2 271.6679159090139 96.937339727378
                            , _bodyRotation = 8.327353346932248e-2
                            , _bodyLinearVelocity = V2 55.54508644587681 (-2.0502524606742365)
                            , _bodyAngularVelocity = 7.142699621333083e-2
                            , _bodyLinearForce = V2 100 0
                            , _bodyTorque = V2 0 0
                            , _bodyMass = Just 10
                            , _bodyShape = Rectangle 100 100
                            }
             testBody2 = RigidBody {
                              _bodyPosition = V2 365.9708409098729 180.626602726224
                            , _bodyRotation = -0.127302834111011
                            , _bodyLinearVelocity = V2 49.549135541237824 20.502524606742373
                            , _bodyAngularVelocity = -0.36039209074250184
                            , _bodyLinearForce = V2 100 0
                            , _bodyTorque = V2 0 0
                            , _bodyMass = Just 10
                            , _bodyShape = Rectangle 100 100
                            }
         in if ((gd ^. collContactPoints) !! 0) `distance` (V2 310.02748096777106 137.37916952622425) > 0.1 || ((gd ^. collContactPoints) !! 1) `distance` (V2 311.7008164411722 150.4526123847951) > 0.1
              then putStrLn ("Error in gcTest: " <> show ((gd ^. collContactPoints) !! 0) <> ", " <> show ((gd ^. collContactPoints) !! 1)) >> return False
              else return True

gcTest2 = let Just gd = generateCollisionData testBody1 testBody2
              testBody1 = RigidBody {
                              _bodyPosition = V2 211.95905202966665 99.27362428995211
                            , _bodyRotation = -2.049046059942099e-2
                            , _bodyLinearVelocity = V2 45.52994180413638 (-1.6701725940347658)
                            , _bodyAngularVelocity = -4.715452921640557e-2
                            , _bodyLinearForce = V2 100 0
                            , _bodyTorque = V2 0 0
                            , _bodyMass = Just 10
                            , _bodyShape = Rectangle 100 100
                            }
              testBody2 = RigidBody {
                              _bodyPosition = V2 313.3194797033412 157.26375710047827
                            , _bodyRotation = 0.6790267257789174
                            , _bodyLinearVelocity = V2 30.700581958640683 16.70172594034766
                            , _bodyAngularVelocity = 0.4124546372446798
                            , _bodyLinearForce = V2 100 0
                            , _bodyTorque = V2 0 0
                            , _bodyMass = Just 10
                            , _bodyShape = Rectangle 100 100
                            }
         in if ((gd ^. collContactPoints) !! 0) `distance` (V2 262.9730072614697 148.23867684386641) > 0.1
              then putStrLn ("Error in gcTest: " <> show ((gd ^. collContactPoints) !! 0)) >> return False
              else return True

unitTests :: IO Bool
unitTests = and <$> sequence [gcTest2]
--unitTests = and <$> sequence [firstCpTest,secondCpTest,thirdCpTest,gcTest]

main :: IO ()
main = do
  testResult <- unitTests
  when testResult $ putStrLn "Tests passed"
  unless testResult $ putStrLn "Tests failed"
  print $ rectangleLines (V2 150 150) (-1.57) (Rectangle 50 50)
{-  putStrLn "OLD (12,5) (8,5)"
  print $ findContactPoints [V2 8 4,V2 14 4,V2 14 9,V2 8 14] [V2 4 2,V2 12 2,V2 12 5,V2 4 5] (V2 0 (-1))
  putStrLn "NEW (6,4)"
  print $ findContactPoints (reverse [V2 2 8,V2 5 11,V2 9 7,V2 6 4]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 0 (-1))
  putStrLn "NEW 2 (12,5) (9.28,5)"
  print $ findContactPoints (reverse [V2 9 4,V2 10 8,V2 14 7,V2 13 3]) [V2 4 2,V2 4 5,V2 12 5,V2 12 2] (V2 (-0.19) (-0.98))-}
--   print $ generateCollisionData testBody1 testBody2

--   withFontInit $ withImgInit $ withWindow "racie 0.0.1.1" $ \window -> do
--       currentTicks <- getTicks
--       stdFont <- SDLTtf.openFont (mediaDir </> "font.ttf") 15
--       withRenderer window screenWidth screenHeight $ \rend -> void $ runStateT mainLoop (initialGameState rend stdFont currentTicks)
