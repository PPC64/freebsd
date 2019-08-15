// AnalysisDeclContext.h - Analysis context for Path Sens analysis -*- C++ -*-//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines AnalysisDeclContext, a class that manages the analysis
// context data for path sensitive analysis.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_ANALYSIS_ANALYSISDECLCONTEXT_H
#define LLVM_CLANG_ANALYSIS_ANALYSISDECLCONTEXT_H

#include "clang/AST/DeclBase.h"
#include "clang/Analysis/BodyFarm.h"
#include "clang/Analysis/CFG.h"
#include "clang/Analysis/CodeInjector.h"
#include "clang/Basic/LLVM.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Support/Allocator.h"
#include <functional>
#include <memory>

namespace clang {

class AnalysisDeclContextManager;
class ASTContext;
class BlockDecl;
class BlockInvocationContext;
class CFGReverseBlockReachabilityAnalysis;
class CFGStmtMap;
class ImplicitParamDecl;
class LocationContext;
class LocationContextManager;
class ParentMap;
class StackFrameContext;
class Stmt;
class VarDecl;

/// The base class of a hierarchy of objects representing analyses tied
/// to AnalysisDeclContext.
class ManagedAnalysis {
protected:
  ManagedAnalysis() = default;

public:
  virtual ~ManagedAnalysis();

  // Subclasses need to implement:
  //
  //  static const void *getTag();
  //
  // Which returns a fixed pointer address to distinguish classes of
  // analysis objects.  They also need to implement:
  //
  //  static [Derived*] create(AnalysisDeclContext &Ctx);
  //
  // which creates the analysis object given an AnalysisDeclContext.
};

/// AnalysisDeclContext contains the context data for the function or method
/// under analysis.
class AnalysisDeclContext {
  /// Backpoint to the AnalysisManager object that created this
  /// AnalysisDeclContext. This may be null.
  AnalysisDeclContextManager *Manager;

  const Decl * const D;

  std::unique_ptr<CFG> cfg, completeCFG;
  std::unique_ptr<CFGStmtMap> cfgStmtMap;

  CFG::BuildOptions cfgBuildOptions;
  CFG::BuildOptions::ForcedBlkExprs *forcedBlkExprs = nullptr;

  bool builtCFG = false;
  bool builtCompleteCFG = false;
  std::unique_ptr<ParentMap> PM;
  std::unique_ptr<CFGReverseBlockReachabilityAnalysis> CFA;

  llvm::BumpPtrAllocator A;

  llvm::DenseMap<const BlockDecl *,void *> *ReferencedBlockVars = nullptr;

  void *ManagedAnalyses = nullptr;

public:
  AnalysisDeclContext(AnalysisDeclContextManager *Mgr,
                  const Decl *D);

  AnalysisDeclContext(AnalysisDeclContextManager *Mgr,
                  const Decl *D,
                  const CFG::BuildOptions &BuildOptions);

  ~AnalysisDeclContext();

  ASTContext &getASTContext() const { return D->getASTContext(); }
  const Decl *getDecl() const { return D; }

  /// Return the AnalysisDeclContextManager (if any) that created
  /// this AnalysisDeclContext.
  AnalysisDeclContextManager *getManager() const {
    return Manager;
  }

  /// Return the build options used to construct the CFG.
  CFG::BuildOptions &getCFGBuildOptions() {
    return cfgBuildOptions;
  }

  const CFG::BuildOptions &getCFGBuildOptions() const {
    return cfgBuildOptions;
  }

  /// getAddEHEdges - Return true iff we are adding exceptional edges from
  /// callExprs.  If this is false, then try/catch statements and blocks
  /// reachable from them can appear to be dead in the CFG, analysis passes must
  /// cope with that.
  bool getAddEHEdges() const { return cfgBuildOptions.AddEHEdges; }
  bool getUseUnoptimizedCFG() const {
      return !cfgBuildOptions.PruneTriviallyFalseEdges;
  }
  bool getAddImplicitDtors() const { return cfgBuildOptions.AddImplicitDtors; }
  bool getAddInitializers() const { return cfgBuildOptions.AddInitializers; }

  void registerForcedBlockExpression(const Stmt *stmt);
  const CFGBlock *getBlockForRegisteredExpression(const Stmt *stmt);

  /// Get the body of the Declaration.
  Stmt *getBody() const;

  /// Get the body of the Declaration.
  /// \param[out] IsAutosynthesized Specifies if the body is auto-generated
  ///             by the BodyFarm.
  Stmt *getBody(bool &IsAutosynthesized) const;

  /// Checks if the body of the Decl is generated by the BodyFarm.
  ///
  /// Note, the lookup is not free. We are going to call getBody behind
  /// the scenes.
  /// \sa getBody
  bool isBodyAutosynthesized() const;

  /// Checks if the body of the Decl is generated by the BodyFarm from a
  /// model file.
  ///
  /// Note, the lookup is not free. We are going to call getBody behind
  /// the scenes.
  /// \sa getBody
  bool isBodyAutosynthesizedFromModelFile() const;

  CFG *getCFG();

  CFGStmtMap *getCFGStmtMap();

  CFGReverseBlockReachabilityAnalysis *getCFGReachablityAnalysis();

  /// Return a version of the CFG without any edges pruned.
  CFG *getUnoptimizedCFG();

  void dumpCFG(bool ShowColors);

  /// Returns true if we have built a CFG for this analysis context.
  /// Note that this doesn't correspond to whether or not a valid CFG exists, it
  /// corresponds to whether we *attempted* to build one.
  bool isCFGBuilt() const { return builtCFG; }

  ParentMap &getParentMap();

  using referenced_decls_iterator = const VarDecl * const *;

  llvm::iterator_range<referenced_decls_iterator>
  getReferencedBlockVars(const BlockDecl *BD);

  /// Return the ImplicitParamDecl* associated with 'self' if this
  /// AnalysisDeclContext wraps an ObjCMethodDecl.  Returns NULL otherwise.
  const ImplicitParamDecl *getSelfDecl() const;

  const StackFrameContext *getStackFrame(LocationContext const *Parent,
                                         const Stmt *S,
                                         const CFGBlock *Blk,
                                         unsigned Idx);

  const BlockInvocationContext *
  getBlockInvocationContext(const LocationContext *parent,
                            const BlockDecl *BD,
                            const void *ContextData);

  /// Return the specified analysis object, lazily running the analysis if
  /// necessary.  Return NULL if the analysis could not run.
  template <typename T>
  T *getAnalysis() {
    const void *tag = T::getTag();
    ManagedAnalysis *&data = getAnalysisImpl(tag);
    if (!data) {
      data = T::create(*this);
    }
    return static_cast<T *>(data);
  }

  /// Returns true if the root namespace of the given declaration is the 'std'
  /// C++ namespace.
  static bool isInStdNamespace(const Decl *D);

private:
  ManagedAnalysis *&getAnalysisImpl(const void* tag);

  LocationContextManager &getLocationContextManager();
};

class LocationContext : public llvm::FoldingSetNode {
public:
  enum ContextKind { StackFrame, Scope, Block };

private:
  ContextKind Kind;

  // AnalysisDeclContext can't be const since some methods may modify its
  // member.
  AnalysisDeclContext *Ctx;

  const LocationContext *Parent;
  int64_t ID;

protected:
  LocationContext(ContextKind k, AnalysisDeclContext *ctx,
                  const LocationContext *parent,
                  int64_t ID)
      : Kind(k), Ctx(ctx), Parent(parent), ID(ID) {}

public:
  virtual ~LocationContext();

  ContextKind getKind() const { return Kind; }

  int64_t getID() const {
    return ID;
  }

  AnalysisDeclContext *getAnalysisDeclContext() const { return Ctx; }

  const LocationContext *getParent() const { return Parent; }

  bool isParentOf(const LocationContext *LC) const;

  const Decl *getDecl() const { return getAnalysisDeclContext()->getDecl(); }

  CFG *getCFG() const { return getAnalysisDeclContext()->getCFG(); }

  template <typename T>
  T *getAnalysis() const {
    return getAnalysisDeclContext()->getAnalysis<T>();
  }

  ParentMap &getParentMap() const {
    return getAnalysisDeclContext()->getParentMap();
  }

  const ImplicitParamDecl *getSelfDecl() const {
    return Ctx->getSelfDecl();
  }

  const StackFrameContext *getStackFrame() const;

  /// Return true if the current LocationContext has no caller context.
  virtual bool inTopFrame() const;

  virtual void Profile(llvm::FoldingSetNodeID &ID) = 0;

  void dumpStack(
      raw_ostream &OS, StringRef Indent = {}, const char *NL = "\n",
      const char *Sep = "",
      std::function<void(const LocationContext *)> printMoreInfoPerContext =
          [](const LocationContext *) {}) const;
  void dumpStack() const;

public:
  static void ProfileCommon(llvm::FoldingSetNodeID &ID,
                            ContextKind ck,
                            AnalysisDeclContext *ctx,
                            const LocationContext *parent,
                            const void *data);
};

class StackFrameContext : public LocationContext {
  friend class LocationContextManager;

  // The callsite where this stack frame is established.
  const Stmt *CallSite;

  // The parent block of the callsite.
  const CFGBlock *Block;

  // The index of the callsite in the CFGBlock.
  unsigned Index;

  StackFrameContext(AnalysisDeclContext *ctx, const LocationContext *parent,
                    const Stmt *s, const CFGBlock *blk,
                    unsigned idx,
                    int64_t ID)
      : LocationContext(StackFrame, ctx, parent, ID), CallSite(s),
        Block(blk), Index(idx) {}

public:
  ~StackFrameContext() override = default;

  const Stmt *getCallSite() const { return CallSite; }

  const CFGBlock *getCallSiteBlock() const { return Block; }

  /// Return true if the current LocationContext has no caller context.
  bool inTopFrame() const override { return getParent() == nullptr;  }

  unsigned getIndex() const { return Index; }

  void Profile(llvm::FoldingSetNodeID &ID) override;

  static void Profile(llvm::FoldingSetNodeID &ID, AnalysisDeclContext *ctx,
                      const LocationContext *parent, const Stmt *s,
                      const CFGBlock *blk, unsigned idx) {
    ProfileCommon(ID, StackFrame, ctx, parent, s);
    ID.AddPointer(blk);
    ID.AddInteger(idx);
  }

  static bool classof(const LocationContext *Ctx) {
    return Ctx->getKind() == StackFrame;
  }
};

class ScopeContext : public LocationContext {
  friend class LocationContextManager;

  const Stmt *Enter;

  ScopeContext(AnalysisDeclContext *ctx, const LocationContext *parent,
               const Stmt *s, int64_t ID)
      : LocationContext(Scope, ctx, parent, ID), Enter(s) {}

public:
  ~ScopeContext() override = default;

  void Profile(llvm::FoldingSetNodeID &ID) override;

  static void Profile(llvm::FoldingSetNodeID &ID, AnalysisDeclContext *ctx,
                      const LocationContext *parent, const Stmt *s) {
    ProfileCommon(ID, Scope, ctx, parent, s);
  }

  static bool classof(const LocationContext *Ctx) {
    return Ctx->getKind() == Scope;
  }
};

class BlockInvocationContext : public LocationContext {
  friend class LocationContextManager;

  const BlockDecl *BD;

  // FIXME: Come up with a more type-safe way to model context-sensitivity.
  const void *ContextData;

  BlockInvocationContext(AnalysisDeclContext *ctx,
                         const LocationContext *parent, const BlockDecl *bd,
                         const void *contextData, int64_t ID)
      : LocationContext(Block, ctx, parent, ID), BD(bd),
        ContextData(contextData) {}

public:
  ~BlockInvocationContext() override = default;

  const BlockDecl *getBlockDecl() const { return BD; }

  const void *getContextData() const { return ContextData; }

  void Profile(llvm::FoldingSetNodeID &ID) override;

  static void Profile(llvm::FoldingSetNodeID &ID, AnalysisDeclContext *ctx,
                      const LocationContext *parent, const BlockDecl *bd,
                      const void *contextData) {
    ProfileCommon(ID, Block, ctx, parent, bd);
    ID.AddPointer(contextData);
  }

  static bool classof(const LocationContext *Ctx) {
    return Ctx->getKind() == Block;
  }
};

class LocationContextManager {
  llvm::FoldingSet<LocationContext> Contexts;

  /// ID used for generating a new location context.
  int64_t NewID = 0;

public:
  ~LocationContextManager();

  const StackFrameContext *getStackFrame(AnalysisDeclContext *ctx,
                                         const LocationContext *parent,
                                         const Stmt *s,
                                         const CFGBlock *blk, unsigned idx);

  const ScopeContext *getScope(AnalysisDeclContext *ctx,
                               const LocationContext *parent,
                               const Stmt *s);

  const BlockInvocationContext *
  getBlockInvocationContext(AnalysisDeclContext *ctx,
                            const LocationContext *parent,
                            const BlockDecl *BD,
                            const void *ContextData);

  /// Discard all previously created LocationContext objects.
  void clear();
private:
  template <typename LOC, typename DATA>
  const LOC *getLocationContext(AnalysisDeclContext *ctx,
                                const LocationContext *parent,
                                const DATA *d);
};

class AnalysisDeclContextManager {
  using ContextMap =
      llvm::DenseMap<const Decl *, std::unique_ptr<AnalysisDeclContext>>;

  ContextMap Contexts;
  LocationContextManager LocContexts;
  CFG::BuildOptions cfgBuildOptions;

  /// Pointer to an interface that can provide function bodies for
  /// declarations from external source.
  std::unique_ptr<CodeInjector> Injector;

  /// A factory for creating and caching implementations for common
  /// methods during the analysis.
  BodyFarm FunctionBodyFarm;

  /// Flag to indicate whether or not bodies should be synthesized
  /// for well-known functions.
  bool SynthesizeBodies;

public:
  AnalysisDeclContextManager(ASTContext &ASTCtx, bool useUnoptimizedCFG = false,
                             bool addImplicitDtors = false,
                             bool addInitializers = false,
                             bool addTemporaryDtors = false,
                             bool addLifetime = false,
                             bool addLoopExit = false,
                             bool addScopes = false,
                             bool synthesizeBodies = false,
                             bool addStaticInitBranches = false,
                             bool addCXXNewAllocator = true,
                             bool addRichCXXConstructors = true,
                             bool markElidedCXXConstructors = true,
                             CodeInjector *injector = nullptr);

  AnalysisDeclContext *getContext(const Decl *D);

  bool getUseUnoptimizedCFG() const {
    return !cfgBuildOptions.PruneTriviallyFalseEdges;
  }

  CFG::BuildOptions &getCFGBuildOptions() {
    return cfgBuildOptions;
  }

  /// Return true if faux bodies should be synthesized for well-known
  /// functions.
  bool synthesizeBodies() const { return SynthesizeBodies; }

  const StackFrameContext *getStackFrame(AnalysisDeclContext *Ctx,
                                         LocationContext const *Parent,
                                         const Stmt *S,
                                         const CFGBlock *Blk,
                                         unsigned Idx) {
    return LocContexts.getStackFrame(Ctx, Parent, S, Blk, Idx);
  }

  // Get the top level stack frame.
  const StackFrameContext *getStackFrame(const Decl *D) {
    return LocContexts.getStackFrame(getContext(D), nullptr, nullptr, nullptr,
                                     0);
  }

  // Get a stack frame with parent.
  StackFrameContext const *getStackFrame(const Decl *D,
                                         LocationContext const *Parent,
                                         const Stmt *S,
                                         const CFGBlock *Blk,
                                         unsigned Idx) {
    return LocContexts.getStackFrame(getContext(D), Parent, S, Blk, Idx);
  }

  /// Get a reference to {@code BodyFarm} instance.
  BodyFarm &getBodyFarm();

  /// Discard all previously created AnalysisDeclContexts.
  void clear();

private:
  friend class AnalysisDeclContext;

  LocationContextManager &getLocationContextManager() {
    return LocContexts;
  }
};

} // namespace clang

#endif // LLVM_CLANG_ANALYSIS_ANALYSISDECLCONTEXT_H
